#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace audio_capture
{
// SNR gradient 전용 제어 흐름:
//   DISABLED
//      enable=true
//          -> INITIAL_SEARCH: 전진+yaw 명령으로 원형/호 궤적 생성
//          -> HOMING: 유효한 방향과 신뢰도가 일정 시간 유지되면 음원 방향 추종
//          -> REACQUIRE: 방향을 잃으면 반대 회전 탐색으로 다시 공간 표본 수집
//          -> HOMING: 방향을 다시 획득하면 추종 재개
class SnrGradientHomingControllerNode : public rclcpp::Node
{
public:
    explicit SnrGradientHomingControllerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("snr_gradient_homing_controller", options)
    {
        // ROS 입출력 설정.
        direction_topic_ =
            declare_parameter<std::string>("direction_topic", "/homing/direction");
        confidence_topic_ =
            declare_parameter<std::string>("confidence_topic", "/homing/snr_confidence");
        enable_topic_ =
            declare_parameter<std::string>("enable_topic", "/homing/control_enable");
        state_topic_ =
            declare_parameter<std::string>("state_topic", "/homing/control_state");
        rc_override_topic_ =
            declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");
        required_direction_frame_ =
            declare_parameter<std::string>("required_direction_frame", "base_link");

        // 상태 전환과 방향 유효성 설정.
        control_enabled_ = declare_parameter<bool>("control_enabled", false);
        rate_hz_ = clamp(declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);
        direction_timeout_s_ =
            clamp(declare_parameter<double>("direction_timeout_s", 0.8), 0.05, 10.0);
        confidence_timeout_s_ =
            clamp(declare_parameter<double>("confidence_timeout_s", 0.8), 0.05, 10.0);
        min_direction_confidence_ =
            clamp(declare_parameter<double>("min_direction_confidence", 0.15), 0.0, 1.0);
        acquire_hold_s_ =
            clamp(declare_parameter<double>("acquire_hold_s", 1.0), 0.0, 30.0);
        search_min_duration_s_ =
            clamp(declare_parameter<double>("search_min_duration_s", 6.0), 0.0, 120.0);
        direction_loss_hold_s_ =
            clamp(declare_parameter<double>("direction_loss_hold_s", 1.0), 0.0, 30.0);

        // 초기/재탐색 원형 운동 설정. 부호는 실제 기체 yaw 채널 방향에 맞춘다.
        search_forward_ =
            clamp(declare_parameter<double>("search_forward", 0.30), 0.0, 1.0);
        search_yaw_ =
            clamp(declare_parameter<double>("search_yaw", 0.30), 0.0, 1.0);
        search_turn_sign_ =
            declare_parameter<double>("search_turn_sign", 1.0) < 0.0 ? -1.0 : 1.0;
        alternate_search_direction_ =
            declare_parameter<bool>("alternate_search_direction", true);

        // 방향 추종 설정. direction은 base_link 기준 단위 벡터를 기대한다.
        forward_fast_ = clamp(declare_parameter<double>("forward_fast", 0.70), 0.0, 1.0);
        forward_mid_ = clamp(declare_parameter<double>("forward_mid", 0.45), 0.0, 1.0);
        forward_slow_ = clamp(declare_parameter<double>("forward_slow", 0.20), 0.0, 1.0);
        yaw_gain_ = clamp(declare_parameter<double>("yaw_gain", 1.15), 0.0, 4.0);
        yaw_limit_ = clamp(declare_parameter<double>("yaw_limit", 0.72), 0.0, 1.0);
        heave_gain_ = clamp(declare_parameter<double>("heave_gain", 0.42), 0.0, 2.0);
        heave_limit_ = clamp(declare_parameter<double>("heave_limit", 0.38), 0.0, 1.0);
        center_deadband_rad_ =
            clamp(declare_parameter<double>("center_deadband_rad", 0.055), 0.0, 0.50);
        confidence_speed_floor_ =
            clamp(declare_parameter<double>("confidence_speed_floor", 0.40), 0.0, 1.0);

        // RC 채널 변환 설정.
        rc_pwm_span_ = clamp(declare_parameter<double>("rc_pwm_span", 400.0), 50.0, 700.0);
        invert_rc_heave_ = declare_parameter<bool>("invert_rc_heave", true);
        invert_rc_yaw_ = declare_parameter<bool>("invert_rc_yaw", true);

        direction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            direction_topic_,
            10,
            std::bind(
                &SnrGradientHomingControllerNode::direction_callback,
                this,
                std::placeholders::_1));
        confidence_sub_ = create_subscription<std_msgs::msg::Float64>(
            confidence_topic_,
            10,
            std::bind(
                &SnrGradientHomingControllerNode::confidence_callback,
                this,
                std::placeholders::_1));
        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            enable_topic_,
            10,
            std::bind(
                &SnrGradientHomingControllerNode::enable_callback,
                this,
                std::placeholders::_1));

        rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, 10);
        state_pub_ = create_publisher<std_msgs::msg::String>(
            state_topic_, rclcpp::QoS(1).reliable().transient_local());

        state_ = control_enabled_ ? State::INITIAL_SEARCH : State::DISABLED;
        state_enter_time_ = now();
        publish_state();

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SnrGradientHomingControllerNode::control_loop, this));

        RCLCPP_INFO(
            get_logger(),
            "SNR gradient controller ready. enabled=%s direction=%s frame=%s rc=%s",
            control_enabled_ ? "true" : "false",
            direction_topic_.c_str(),
            required_direction_frame_.c_str(),
            rc_override_topic_.c_str());
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr std::uint16_t RC_NEUTRAL = 1500;
    static constexpr std::size_t PITCH_CHANNEL_INDEX = 0;
    static constexpr std::size_t ROLL_CHANNEL_INDEX = 1;
    static constexpr std::size_t VERTICAL_CHANNEL_INDEX = 2;
    static constexpr std::size_t YAW_CHANNEL_INDEX = 3;
    static constexpr std::size_t FORWARD_CHANNEL_INDEX = 4;
    static constexpr std::size_t LATERAL_CHANNEL_INDEX = 5;
    static constexpr std::size_t PRIMARY_CHANNEL_COUNT = 8;

    enum class State
    {
        DISABLED,
        INITIAL_SEARCH,
        HOMING,
        REACQUIRE
    };

    struct Command
    {
        double forward = 0.0;
        double sway = 0.0;
        double heave = 0.0;
        double yaw = 0.0;
    };

    void direction_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        // 좌표계가 다르면 body-frame 제어 입력으로 사용할 수 없다.
        if (!required_direction_frame_.empty() &&
            msg->header.frame_id != required_direction_frame_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Direction frame '%s' does not match required frame '%s'.",
                msg->header.frame_id.c_str(),
                required_direction_frame_.c_str());
            return;
        }

        const double x = msg->vector.x;
        const double y = msg->vector.y;
        const double z = msg->vector.z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            return;
        }

        const double norm = std::sqrt(x * x + y * y + z * z);
        if (norm < 1.0e-6) {
            return;
        }

        direction_x_ = x / norm;
        direction_y_ = y / norm;
        direction_z_ = z / norm;
        last_direction_time_ = now();
        have_direction_ = true;
    }

    void confidence_callback(const std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        if (!std::isfinite(msg->data)) {
            return;
        }
        direction_confidence_ = clamp(msg->data, 0.0, 1.0);
        last_confidence_time_ = now();
        have_confidence_ = true;
    }

    void enable_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if (msg->data == control_enabled_) {
            return;
        }

        control_enabled_ = msg->data;
        have_acquire_start_ = false;
        have_last_valid_signal_time_ = false;
        if (control_enabled_) {
            // enable 상승 시 항상 새로운 초기 탐색부터 시작한다.
            transition_to(State::INITIAL_SEARCH);
        } else {
            transition_to(State::DISABLED);
        }
    }

    void control_loop()
    {
        const rclcpp::Time current_time = now();

        // 비활성 상태에서는 RC override를 해제하여 수동 조종을 방해하지 않는다.
        if (!control_enabled_ || state_ == State::DISABLED) {
            rc_pub_->publish(make_release_override());
            return;
        }

        const bool signal_valid = have_valid_signal(current_time);

        // 실행 분기 A: 초기 탐색 또는 방향 재탐색.
        if (state_ == State::INITIAL_SEARCH || state_ == State::REACQUIRE) {
            update_search_acquisition(current_time, signal_valid);
            if (state_ == State::HOMING) {
                rc_pub_->publish(make_rc_override(homing_command()));
            } else {
                rc_pub_->publish(make_rc_override(search_command()));
            }
            return;
        }

        // 실행 분기 B: 유효한 방향을 따라 음원 쪽으로 전진한다.
        if (state_ == State::HOMING && signal_valid) {
            last_valid_signal_time_ = current_time;
            have_last_valid_signal_time_ = true;
            rc_pub_->publish(make_rc_override(homing_command()));
            return;
        }

        // 실행 분기 C: 일시적인 방향 손실 동안 잘못된 방향으로 진행하지 않는다.
        rc_pub_->publish(make_rc_override(Command{}));
        if (!have_last_valid_signal_time_) {
            last_valid_signal_time_ = current_time;
            have_last_valid_signal_time_ = true;
        }
        if ((current_time - last_valid_signal_time_).seconds() >= direction_loss_hold_s_) {
            if (alternate_search_direction_) {
                search_turn_sign_ *= -1.0;
            }
            transition_to(State::REACQUIRE);
        }
    }

    void update_search_acquisition(const rclcpp::Time & current_time, const bool signal_valid)
    {
        // 방향이 불안정하면 연속 획득 시간을 처음부터 다시 센다.
        if (!signal_valid) {
            have_acquire_start_ = false;
            return;
        }
        if (!have_acquire_start_) {
            acquire_start_time_ = current_time;
            have_acquire_start_ = true;
        }

        const double search_elapsed_s = (current_time - state_enter_time_).seconds();
        const double acquire_elapsed_s = (current_time - acquire_start_time_).seconds();
        if (search_elapsed_s < search_min_duration_s_ || acquire_elapsed_s < acquire_hold_s_) {
            return;
        }

        last_valid_signal_time_ = current_time;
        have_last_valid_signal_time_ = true;
        transition_to(State::HOMING);
    }

    bool have_valid_signal(const rclcpp::Time & current_time) const
    {
        if (!have_direction_ || !have_confidence_) {
            return false;
        }
        if ((current_time - last_direction_time_).seconds() > direction_timeout_s_ ||
            (current_time - last_confidence_time_).seconds() > confidence_timeout_s_)
        {
            return false;
        }
        return direction_confidence_ >= min_direction_confidence_;
    }

    Command search_command() const
    {
        // 전진과 일정 yaw를 동시에 주어 원형 또는 충분한 곡률의 호를 만든다.
        Command command;
        command.forward = search_forward_;
        command.yaw = search_turn_sign_ * search_yaw_;
        return command;
    }

    Command homing_command() const
    {
        Command command;

        const double bearing = wrap_pi(std::atan2(direction_y_, direction_x_));
        const double absolute_bearing = std::abs(bearing);
        command.yaw = absolute_bearing <= center_deadband_rad_ ?
            0.0 : clamp(yaw_gain_ * bearing, -yaw_limit_, yaw_limit_);
        command.heave = clamp(-heave_gain_ * direction_z_, -heave_limit_, heave_limit_);

        // 방향 오차가 크면 회전을 우선하고 전진 속도를 줄인다.
        double forward = forward_fast_;
        if (absolute_bearing > 1.10) {
            forward = std::min(forward, forward_slow_);
        } else if (absolute_bearing > 0.72) {
            forward = std::min(forward, forward_mid_);
        } else if (absolute_bearing > 0.42) {
            forward = std::min(forward, std::max(forward_mid_, 0.55));
        }
        if (direction_x_ < 0.05) {
            forward = std::min(forward, 0.15);
        }

        // 신뢰도가 낮을수록 전진량을 줄이되 획득 임계값 이상에서는 정지하지 않는다.
        const double confidence_scale = confidence_speed_floor_ +
            (1.0 - confidence_speed_floor_) * direction_confidence_;
        command.forward = clamp(forward * confidence_scale, 0.0, 1.0);
        return command;
    }

    mavros_msgs::msg::OverrideRCIn make_rc_override(const Command & command) const
    {
        mavros_msgs::msg::OverrideRCIn msg;
        msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
        for (std::size_t index = 0;
            index < PRIMARY_CHANNEL_COUNT && index < msg.channels.size();
            ++index)
        {
            msg.channels[index] = RC_NEUTRAL;
        }

        msg.channels[PITCH_CHANNEL_INDEX] = RC_NEUTRAL;
        msg.channels[ROLL_CHANNEL_INDEX] = RC_NEUTRAL;
        msg.channels[VERTICAL_CHANNEL_INDEX] = axis_pwm(command.heave, invert_rc_heave_);
        msg.channels[YAW_CHANNEL_INDEX] = axis_pwm(command.yaw, invert_rc_yaw_);
        msg.channels[FORWARD_CHANNEL_INDEX] = axis_pwm(command.forward, false);
        msg.channels[LATERAL_CHANNEL_INDEX] = axis_pwm(command.sway, false);
        return msg;
    }

    mavros_msgs::msg::OverrideRCIn make_release_override() const
    {
        mavros_msgs::msg::OverrideRCIn msg;
        msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
        return msg;
    }

    void transition_to(const State next_state)
    {
        if (state_ == next_state) {
            return;
        }
        state_ = next_state;
        state_enter_time_ = now();
        have_acquire_start_ = false;
        publish_state();
        RCLCPP_INFO(get_logger(), "Homing control state -> %s", state_name(state_));
    }

    void publish_state()
    {
        std_msgs::msg::String msg;
        msg.data = state_name(state_);
        state_pub_->publish(msg);
    }

    static const char * state_name(const State state)
    {
        switch (state) {
            case State::DISABLED:
                return "DISABLED";
            case State::INITIAL_SEARCH:
                return "INITIAL_SEARCH";
            case State::HOMING:
                return "HOMING";
            case State::REACQUIRE:
                return "REACQUIRE";
        }
        return "UNKNOWN";
    }

    std::uint16_t axis_pwm(const double value, const bool invert) const
    {
        const double axis = invert ? -value : value;
        const int pwm = static_cast<int>(std::llround(
            static_cast<double>(RC_NEUTRAL) + clamp(axis, -1.0, 1.0) * rc_pwm_span_));
        return static_cast<std::uint16_t>(std::clamp(pwm, 1100, 1900));
    }

    static double clamp(const double value, const double low, const double high)
    {
        return std::max(low, std::min(value, high));
    }

    static double wrap_pi(double angle)
    {
        while (angle > PI) {
            angle -= 2.0 * PI;
        }
        while (angle < -PI) {
            angle += 2.0 * PI;
        }
        return angle;
    }

    std::string direction_topic_;
    std::string confidence_topic_;
    std::string enable_topic_;
    std::string state_topic_;
    std::string rc_override_topic_;
    std::string required_direction_frame_;

    bool control_enabled_ = false;
    double rate_hz_ = 30.0;
    double direction_timeout_s_ = 0.8;
    double confidence_timeout_s_ = 0.8;
    double min_direction_confidence_ = 0.15;
    double acquire_hold_s_ = 1.0;
    double search_min_duration_s_ = 6.0;
    double direction_loss_hold_s_ = 1.0;

    double search_forward_ = 0.30;
    double search_yaw_ = 0.30;
    double search_turn_sign_ = 1.0;
    bool alternate_search_direction_ = true;

    double forward_fast_ = 0.70;
    double forward_mid_ = 0.45;
    double forward_slow_ = 0.20;
    double yaw_gain_ = 1.15;
    double yaw_limit_ = 0.72;
    double heave_gain_ = 0.42;
    double heave_limit_ = 0.38;
    double center_deadband_rad_ = 0.055;
    double confidence_speed_floor_ = 0.40;
    double rc_pwm_span_ = 400.0;
    bool invert_rc_heave_ = true;
    bool invert_rc_yaw_ = true;

    State state_ = State::DISABLED;
    rclcpp::Time state_enter_time_;
    rclcpp::Time acquire_start_time_;
    bool have_acquire_start_ = false;
    rclcpp::Time last_valid_signal_time_;
    bool have_last_valid_signal_time_ = false;

    double direction_x_ = 1.0;
    double direction_y_ = 0.0;
    double direction_z_ = 0.0;
    double direction_confidence_ = 0.0;
    rclcpp::Time last_direction_time_;
    rclcpp::Time last_confidence_time_;
    bool have_direction_ = false;
    bool have_confidence_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr confidence_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::SnrGradientHomingControllerNode)
