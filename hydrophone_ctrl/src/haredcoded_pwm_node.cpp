#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <audio_common_msgs/msg/float64_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace hydrophone_ctrl
{

class HaredcodedPwmNode : public rclcpp::Node
{
public:
    HaredcodedPwmNode()
    : Node("haredcoded_pwm_node")
    {
        first_forward_pwm_ = declare_parameter<int>("first_forward_pwm", 1600);
        first_forward_duration_sec_ = declare_parameter<double>(
            "first_forward_duration_sec", 3.0);
        yaw_pwm_ = declare_parameter<int>("yaw_pwm", 1600);
        yaw_duration_sec_ = declare_parameter<double>("yaw_duration_sec", 1.0);
        second_forward_pwm_ = declare_parameter<int>("second_forward_pwm", 1600);
        second_forward_duration_sec_ = declare_parameter<double>(
            "second_forward_duration_sec", 3.0);
        clockwise_yaw_pwm_ = declare_parameter<int>("clockwise_yaw_pwm", 1600);
        clockwise_yaw_duration_sec_ = declare_parameter<double>(
            "clockwise_yaw_duration_sec", 1.0);
        snr_forward_pwm_ = declare_parameter<int>("snr_forward_pwm", 1600);
        snr_forward_duration_sec_ = declare_parameter<double>(
            "snr_forward_duration_sec", 5.0);
        uturn_yaw_pwm_ = declare_parameter<int>("uturn_yaw_pwm", 1600);
        uturn_yaw_duration_sec_ = declare_parameter<double>(
            "uturn_yaw_duration_sec", 2.0);
        return_forward_pwm_ = declare_parameter<int>("return_forward_pwm", 1600);
        snr_sample_period_sec_ = declare_parameter<double>(
            "snr_sample_period_sec", 0.1);
        snr_median_window_size_ = declare_parameter<int>(
            "snr_median_window_size", 3);
        global_max_region_window_size_ = declare_parameter<int>(
            "global_max_region_window_size", 5);
        const auto snr_topic = declare_parameter<std::string>(
            "snr_topic", "/audio_frequency_detector/snr_db_stamped");
        const auto vision_search_request_topic = declare_parameter<std::string>(
            "vision_search_request_topic", "/homing/vision_search_active");
        const auto target_confirmed_topic = declare_parameter<std::string>(
            "target_confirmed_topic", "/vision/target_confirmed");
        const auto vision_control_granted_topic = declare_parameter<std::string>(
            "vision_control_granted_topic", "/homing/vision_control_granted");
        const auto timeout_topic = declare_parameter<std::string>(
            "timeout_topic", "/homing/timeout");
        const auto state_topic = declare_parameter<std::string>(
            "state_topic", "/homing/haredcoded_pwm_state");
        const auto raw_snr_map_topic = declare_parameter<std::string>(
            "raw_snr_map_topic", "/homing/snr_raw_time_map");
        const auto filtered_snr_map_topic = declare_parameter<std::string>(
            "filtered_snr_map_topic", "/homing/snr_filtered_time_map");
        const auto selected_snr_target_topic = declare_parameter<std::string>(
            "selected_snr_target_topic", "/homing/snr_selected_time_target");
        vision_confirmation_lead_sec_ = declare_parameter<double>(
            "vision_confirmation_lead_sec", 1.0);
        const auto odometry_topic = declare_parameter<std::string>(
            "odometry_topic", "/odometry/filtered");
        target_depth_z_m_ = declare_parameter<double>("target_depth_z_m", -8.0);
        depth_kp_ = std::max(0.0, declare_parameter<double>("depth_kp", 0.8));
        depth_ki_ = std::max(0.0, declare_parameter<double>("depth_ki", 0.15));
        depth_kd_ = std::max(0.0, declare_parameter<double>("depth_kd", 0.0));
        depth_bias_ = declare_parameter<double>("depth_bias", 0.0);
        depth_integral_limit_ = std::max(
            0.0, declare_parameter<double>("depth_integral_limit", 2.0));
        heave_limit_ = std::clamp(
            declare_parameter<double>("heave_limit", 0.2), 0.0, 1.0);
        rc_pwm_span_ = std::clamp(
            declare_parameter<double>("rc_pwm_span", 400.0), 50.0, 700.0);

        validate_parameters();

        rc_publisher_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(
            RC_OVERRIDE_TOPIC, 10);
        snr_subscription_ =
            create_subscription<audio_common_msgs::msg::Float64Stamped>(
                snr_topic, 20,
                [this](
                    const audio_common_msgs::msg::Float64Stamped::ConstSharedPtr msg)
                {
                    if (phase_ == Phase::SNR_FORWARD && std::isfinite(msg->data)) {
                        latest_snr_db_ = msg->data;
                        have_latest_snr_ = true;
                    }
                });
        target_confirmed_subscription_ = create_subscription<std_msgs::msg::Bool>(
            target_confirmed_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Bool::ConstSharedPtr msg)
            {
                if (msg->data && phase_ == Phase::RETURN_TO_MAX_SNR_TIME &&
                    vision_search_requested_)
                {
                    begin_vision_handoff("target_confirmed", false);
                }
            });
        odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 30,
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                const double z_m = msg->pose.pose.position.z;
                if (!std::isfinite(z_m)) {
                    return;
                }
                current_z_m_ = z_m;
                have_odometry_ = true;
            });
        vision_search_request_publisher_ = create_publisher<std_msgs::msg::Bool>(
            vision_search_request_topic,
            rclcpp::QoS(1).reliable().transient_local());
        vision_control_granted_publisher_ = create_publisher<std_msgs::msg::Bool>(
            vision_control_granted_topic,
            rclcpp::QoS(1).reliable().transient_local());
        timeout_publisher_ = create_publisher<std_msgs::msg::Bool>(
            timeout_topic, rclcpp::QoS(1).reliable().transient_local());
        state_publisher_ = create_publisher<std_msgs::msg::String>(
            state_topic, rclcpp::QoS(1).reliable().transient_local());
        raw_snr_map_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(
            raw_snr_map_topic, 20);
        filtered_snr_map_publisher_ =
            create_publisher<geometry_msgs::msg::PointStamped>(
                filtered_snr_map_topic, 20);
        selected_snr_target_publisher_ =
            create_publisher<geometry_msgs::msg::PointStamped>(
                selected_snr_target_topic,
                rclcpp::QoS(1).reliable().transient_local());
        publish_bool(vision_search_request_publisher_, false);
        publish_bool(vision_control_granted_publisher_, false);
        publish_bool(timeout_publisher_, false);
        publish_state();
        phase_start_time_ = now();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_PERIOD_MS),
            [this]() {control_loop();});

        RCLCPP_WARN(
            get_logger(),
            "Timed PWM sequence started: forward(%d, %.2fs) -> "
            "yaw(%d, %.2fs) -> forward(%d, %.2fs) -> clockwise yaw(%d, %.2fs) "
            "-> SNR forward(%d, %.2fs) -> 180 yaw(%d, %.2fs)",
            first_forward_pwm_, first_forward_duration_sec_,
            yaw_pwm_, yaw_duration_sec_,
            second_forward_pwm_, second_forward_duration_sec_,
            clockwise_yaw_pwm_, clockwise_yaw_duration_sec_,
            snr_forward_pwm_, snr_forward_duration_sec_,
            uturn_yaw_pwm_, uturn_yaw_duration_sec_);
    }

    ~HaredcodedPwmNode() override
    {
        for (int count = 0; count < 3; ++count) {
            publish_pwm(RC_NEUTRAL_PWM, RC_NEUTRAL_PWM);
        }
    }

private:
    enum class Phase
    {
        FIRST_FORWARD,
        YAW,
        SECOND_FORWARD,
        CLOCKWISE_YAW,
        SNR_FORWARD,
        UTURN,
        RETURN_TO_MAX_SNR_TIME,
        HANDOFF_NEUTRAL,
        COMPLETE
    };

    struct SnrSample
    {
        double elapsed_sec;
        double snr_db;
    };

    static constexpr const char * RC_OVERRIDE_TOPIC = "/mavros/rc/override";
    static constexpr std::size_t VERTICAL_CHANNEL_INDEX = 2;
    static constexpr std::size_t YAW_CHANNEL_INDEX = 3;
    static constexpr std::size_t FORWARD_CHANNEL_INDEX = 4;
    static constexpr std::uint16_t RC_NEUTRAL_PWM = 1500;
    static constexpr int PWM_MIN = 1100;
    static constexpr int PWM_MAX = 1900;
    static constexpr int PUBLISH_PERIOD_MS = 33;
    static constexpr double PUBLISH_RATE_HZ = 30.0;

    void validate_parameters() const
    {
        validate_pwm("first_forward_pwm", first_forward_pwm_);
        validate_pwm("yaw_pwm", yaw_pwm_);
        validate_pwm("second_forward_pwm", second_forward_pwm_);
        validate_pwm("clockwise_yaw_pwm", clockwise_yaw_pwm_);
        validate_pwm("snr_forward_pwm", snr_forward_pwm_);
        validate_pwm("uturn_yaw_pwm", uturn_yaw_pwm_);
        validate_pwm("return_forward_pwm", return_forward_pwm_);
        validate_duration(
            "first_forward_duration_sec", first_forward_duration_sec_);
        validate_duration("yaw_duration_sec", yaw_duration_sec_);
        validate_duration(
            "second_forward_duration_sec", second_forward_duration_sec_);
        validate_duration(
            "clockwise_yaw_duration_sec", clockwise_yaw_duration_sec_);
        validate_duration("snr_forward_duration_sec", snr_forward_duration_sec_);
        validate_duration("uturn_yaw_duration_sec", uturn_yaw_duration_sec_);
        validate_duration(
            "vision_confirmation_lead_sec", vision_confirmation_lead_sec_);
        if (!std::isfinite(target_depth_z_m_)) {
            throw std::invalid_argument("target_depth_z_m must be finite");
        }
        if (!std::isfinite(depth_bias_)) {
            throw std::invalid_argument("depth_bias must be finite");
        }
        if (!std::isfinite(snr_sample_period_sec_) || snr_sample_period_sec_ <= 0.0) {
            throw std::invalid_argument(
                "snr_sample_period_sec must be finite and greater than zero");
        }
        if (snr_median_window_size_ <= 0 || snr_median_window_size_ % 2 == 0) {
            throw std::invalid_argument(
                "snr_median_window_size must be a positive odd number");
        }
        if (global_max_region_window_size_ <= 0) {
            throw std::invalid_argument(
                "global_max_region_window_size must be greater than zero");
        }
    }

    void validate_pwm(const std::string & name, const int pwm) const
    {
        if (pwm < PWM_MIN || pwm > PWM_MAX) {
            throw std::invalid_argument(
                name + " must be between 1100 and 1900");
        }
    }

    void validate_duration(const std::string & name, const double duration) const
    {
        if (!std::isfinite(duration) || duration < 0.0) {
            throw std::invalid_argument(
                name + " must be finite and zero or greater");
        }
    }

    void control_loop()
    {
        if (phase_ == Phase::HANDOFF_NEUTRAL) {
            control_handoff_neutral();
            return;
        }
        if (phase_ == Phase::COMPLETE) {
            return;
        }

        skip_zero_duration_phases();

        switch (phase_) {
            case Phase::FIRST_FORWARD:
                publish_control_pwm(RC_NEUTRAL_PWM, first_forward_pwm_);
                transition_when_elapsed(
                    first_forward_duration_sec_, Phase::YAW, "YAW");
                return;
            case Phase::YAW:
                publish_control_pwm(yaw_pwm_, RC_NEUTRAL_PWM);
                transition_when_elapsed(
                    yaw_duration_sec_, Phase::SECOND_FORWARD,
                    "SECOND_FORWARD");
                return;
            case Phase::SECOND_FORWARD:
                publish_control_pwm(RC_NEUTRAL_PWM, second_forward_pwm_);
                transition_when_elapsed(
                    second_forward_duration_sec_, Phase::CLOCKWISE_YAW,
                    "CLOCKWISE_YAW");
                return;
            case Phase::CLOCKWISE_YAW:
                publish_control_pwm(clockwise_yaw_pwm_, RC_NEUTRAL_PWM);
                transition_when_elapsed(
                    clockwise_yaw_duration_sec_, Phase::SNR_FORWARD,
                    "SNR_FORWARD");
                return;
            case Phase::SNR_FORWARD:
                publish_control_pwm(RC_NEUTRAL_PWM, snr_forward_pwm_);
                record_snr_sample_if_due();
                transition_when_elapsed(
                    snr_forward_duration_sec_, Phase::UTURN, "UTURN");
                return;
            case Phase::UTURN:
                publish_control_pwm(uturn_yaw_pwm_, RC_NEUTRAL_PWM);
                transition_when_elapsed(
                    uturn_yaw_duration_sec_, Phase::RETURN_TO_MAX_SNR_TIME,
                    "RETURN_TO_MAX_SNR_TIME");
                return;
            case Phase::RETURN_TO_MAX_SNR_TIME:
                control_return_and_handoff();
                return;
            case Phase::HANDOFF_NEUTRAL:
                return;
            case Phase::COMPLETE:
                // Vision owns RC after the handshake completes.
                return;
        }
    }

    void skip_zero_duration_phases()
    {
        while (phase_ != Phase::COMPLETE && current_phase_duration() == 0.0) {
            switch (phase_) {
                case Phase::FIRST_FORWARD:
                    transition_to(Phase::YAW, "YAW");
                    break;
                case Phase::YAW:
                    transition_to(Phase::SECOND_FORWARD, "SECOND_FORWARD");
                    break;
                case Phase::SECOND_FORWARD:
                    transition_to(Phase::CLOCKWISE_YAW, "CLOCKWISE_YAW");
                    break;
                case Phase::CLOCKWISE_YAW:
                    transition_to(Phase::SNR_FORWARD, "SNR_FORWARD");
                    break;
                case Phase::SNR_FORWARD:
                    transition_to(Phase::UTURN, "UTURN");
                    break;
                case Phase::UTURN:
                    transition_to(
                        Phase::RETURN_TO_MAX_SNR_TIME,
                        "RETURN_TO_MAX_SNR_TIME");
                    break;
                case Phase::RETURN_TO_MAX_SNR_TIME:
                    request_vision_confirmation();
                    begin_vision_handoff("return_timeout", true);
                    return;
                case Phase::HANDOFF_NEUTRAL:
                    return;
                case Phase::COMPLETE:
                    return;
            }
        }
    }

    double current_phase_duration() const
    {
        switch (phase_) {
            case Phase::FIRST_FORWARD:
                return first_forward_duration_sec_;
            case Phase::YAW:
                return yaw_duration_sec_;
            case Phase::SECOND_FORWARD:
                return second_forward_duration_sec_;
            case Phase::CLOCKWISE_YAW:
                return clockwise_yaw_duration_sec_;
            case Phase::SNR_FORWARD:
                return snr_forward_duration_sec_;
            case Phase::UTURN:
                return uturn_yaw_duration_sec_;
            case Phase::RETURN_TO_MAX_SNR_TIME:
                return return_forward_duration_sec_;
            case Phase::HANDOFF_NEUTRAL:
                return 1.0 / PUBLISH_RATE_HZ;
            case Phase::COMPLETE:
                return 0.0;
        }
        return 0.0;
    }

    void transition_when_elapsed(
        const double duration_sec, const Phase next_phase,
        const char * next_phase_name)
    {
        if ((now() - phase_start_time_).seconds() >= duration_sec) {
            transition_to(next_phase, next_phase_name);
        }
    }

    void transition_to(const Phase next_phase, const char * next_phase_name)
    {
        publish_pwm(RC_NEUTRAL_PWM, RC_NEUTRAL_PWM);
        phase_ = next_phase;
        phase_start_time_ = now();
        publish_state();

        if (next_phase == Phase::SNR_FORWARD) {
            snr_map_.clear();
            snr_filter_window_.clear();
            filtered_snr_profile_.clear();
            have_latest_snr_ = false;
            max_snr_db_ = -std::numeric_limits<double>::infinity();
            max_snr_elapsed_sec_ = 0.0;
            next_snr_sample_elapsed_sec_ = snr_sample_period_sec_;
        } else if (next_phase == Phase::UTURN) {
            const bool selected = select_global_max_region();
            return_forward_duration_sec_ = selected ? max_snr_elapsed_sec_ : 0.0;
            if (!selected) {
                RCLCPP_WARN(
                    get_logger(),
                    "Not enough valid SNR data; return duration is 0 seconds");
            } else {
                RCLCPP_INFO(
                    get_logger(),
                    "SNR map complete: raw=%zu filtered=%zu peak=%.2f dB "
                    "target_time=%.2f s; "
                    "return duration=%.2f s",
                    snr_map_.size(), filtered_snr_profile_.size(),
                    max_snr_db_, max_snr_elapsed_sec_,
                    return_forward_duration_sec_);
            }
        }
        RCLCPP_INFO(get_logger(), "PWM phase -> %s", next_phase_name);
    }

    void control_return_and_handoff()
    {
        const double elapsed_sec = (now() - phase_start_time_).seconds();
        const double remaining_sec = std::max(
            0.0, return_forward_duration_sec_ - elapsed_sec);

        if (!vision_search_requested_ &&
            remaining_sec <= vision_confirmation_lead_sec_)
        {
            request_vision_confirmation();
        }

        if (elapsed_sec >= return_forward_duration_sec_) {
            if (!vision_search_requested_) {
                request_vision_confirmation();
            }
            begin_vision_handoff("return_timeout", true);
            return;
        }
        publish_control_pwm(RC_NEUTRAL_PWM, return_forward_pwm_);
    }

    void request_vision_confirmation()
    {
        if (vision_search_requested_ || phase_ == Phase::HANDOFF_NEUTRAL ||
            phase_ == Phase::COMPLETE)
        {
            return;
        }
        vision_search_requested_ = true;
        publish_bool(vision_control_granted_publisher_, false);
        publish_bool(vision_search_request_publisher_, true);
        RCLCPP_INFO(
            get_logger(),
            "[VISION] target confirmation requested; acoustic PWM control kept");
    }

    void begin_vision_handoff(const char * reason, const bool timed_out)
    {
        if (phase_ == Phase::HANDOFF_NEUTRAL || phase_ == Phase::COMPLETE) {
            return;
        }
        if (!vision_search_requested_) {
            request_vision_confirmation();
        }
        if (timed_out) {
            publish_bool(timeout_publisher_, true);
            RCLCPP_WARN(
                get_logger(),
                "[VISION] confirmation timeout at selected SNR region");
        }
        publish_pwm(RC_NEUTRAL_PWM, RC_NEUTRAL_PWM);
        handoff_neutral_sent_ = false;
        phase_ = Phase::HANDOFF_NEUTRAL;
        phase_start_time_ = now();
        publish_state();
        RCLCPP_INFO(get_logger(), "[VISION] handoff started: reason=%s", reason);
    }

    void control_handoff_neutral()
    {
        if (!handoff_neutral_sent_) {
            publish_pwm(RC_NEUTRAL_PWM, RC_NEUTRAL_PWM);
            handoff_neutral_sent_ = true;
            phase_start_time_ = now();
            return;
        }
        if ((now() - phase_start_time_).seconds() < 1.0 / PUBLISH_RATE_HZ) {
            return;
        }

        phase_ = Phase::COMPLETE;
        publish_state();
        publish_bool(vision_control_granted_publisher_, true);
        RCLCPP_INFO(
            get_logger(),
            "[VISION] acoustic RC stopped; vision control granted");
    }

    static void publish_bool(
        const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher,
        const bool value)
    {
        std_msgs::msg::Bool message;
        message.data = value;
        publisher->publish(message);
    }

    void record_snr_sample_if_due()
    {
        if (!have_latest_snr_) {
            return;
        }

        const double elapsed_sec = (now() - phase_start_time_).seconds();
        if (elapsed_sec < next_snr_sample_elapsed_sec_) {
            return;
        }

        // If callbacks were delayed, skip missed buckets instead of duplicating
        // the same latest SNR value into every old 0.1-second slot.
        while (next_snr_sample_elapsed_sec_ + snr_sample_period_sec_ <= elapsed_sec) {
            next_snr_sample_elapsed_sec_ += snr_sample_period_sec_;
        }
        const double sample_time_sec = next_snr_sample_elapsed_sec_;
        const SnrSample raw_sample{sample_time_sec, latest_snr_db_};
        snr_map_.push_back(raw_sample);
        publish_snr_point(raw_snr_map_publisher_, raw_sample);
        next_snr_sample_elapsed_sec_ += snr_sample_period_sec_;

        snr_filter_window_.push_back(raw_sample);
        const auto median_window_size =
            static_cast<std::size_t>(snr_median_window_size_);
        while (snr_filter_window_.size() > median_window_size) {
            snr_filter_window_.pop_front();
        }
        if (snr_filter_window_.size() < median_window_size) {
            RCLCPP_INFO(
                get_logger(), "[SNR MAP] time=%.2f s raw=%.2f dB (warming filter)",
                sample_time_sec, latest_snr_db_);
            return;
        }

        std::vector<double> values;
        values.reserve(snr_filter_window_.size());
        for (const auto & sample : snr_filter_window_) {
            values.push_back(sample.snr_db);
        }
        const auto middle = values.begin() +
            static_cast<std::ptrdiff_t>(values.size() / 2);
        std::nth_element(values.begin(), middle, values.end());

        const SnrSample filtered_sample{
            snr_filter_window_[snr_filter_window_.size() / 2].elapsed_sec,
            *middle};
        filtered_snr_profile_.push_back(filtered_sample);
        publish_snr_point(filtered_snr_map_publisher_, filtered_sample);
        RCLCPP_INFO(
            get_logger(),
            "[SNR MAP] time=%.2f s raw=%.2f dB median=%.2f dB",
            filtered_sample.elapsed_sec, latest_snr_db_, filtered_sample.snr_db);
    }

    bool select_global_max_region()
    {
        if (filtered_snr_profile_.empty()) {
            return false;
        }

        const std::size_t window_size = std::min(
            static_cast<std::size_t>(global_max_region_window_size_),
            filtered_snr_profile_.size());
        std::size_t best_start = 0;
        double rolling_sum = 0.0;
        for (std::size_t index = 0; index < window_size; ++index) {
            rolling_sum += filtered_snr_profile_[index].snr_db;
        }
        double best_average = rolling_sum / static_cast<double>(window_size);
        for (std::size_t start = 1;
            start + window_size <= filtered_snr_profile_.size(); ++start)
        {
            rolling_sum -= filtered_snr_profile_[start - 1].snr_db;
            rolling_sum +=
                filtered_snr_profile_[start + window_size - 1].snr_db;
            const double average = rolling_sum / static_cast<double>(window_size);
            if (average > best_average) {
                best_average = average;
                best_start = start;
            }
        }

        double region_peak_snr = -std::numeric_limits<double>::infinity();
        for (std::size_t index = best_start;
            index < best_start + window_size; ++index)
        {
            region_peak_snr = std::max(
                region_peak_snr, filtered_snr_profile_[index].snr_db);
        }

        double weighted_time_sec = 0.0;
        double total_weight = 0.0;
        for (std::size_t index = best_start;
            index < best_start + window_size; ++index)
        {
            const auto & sample = filtered_snr_profile_[index];
            const double weight = std::pow(
                10.0, (sample.snr_db - region_peak_snr) / 10.0);
            weighted_time_sec += weight * sample.elapsed_sec;
            total_weight += weight;
        }
        if (total_weight <= 0.0 || !std::isfinite(total_weight)) {
            return false;
        }

        max_snr_db_ = region_peak_snr;
        max_snr_elapsed_sec_ = weighted_time_sec / total_weight;
        publish_snr_point(
            selected_snr_target_publisher_,
            SnrSample{max_snr_elapsed_sec_, max_snr_db_});
        RCLCPP_INFO(
            get_logger(),
            "[SNR] global max region: samples=%zu-%zu/%zu average=%.2f dB "
            "peak=%.2f dB weighted_time=%.2f s",
            best_start, best_start + window_size - 1,
            filtered_snr_profile_.size(), best_average, max_snr_db_,
            max_snr_elapsed_sec_);
        return true;
    }

    void publish_state()
    {
        std_msgs::msg::String message;
        message.data = phase_name(phase_);
        state_publisher_->publish(message);
    }

    void publish_snr_point(
        const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & publisher,
        const SnrSample & sample)
    {
        geometry_msgs::msg::PointStamped message;
        message.header.stamp = now();
        message.header.frame_id = "time_snr";
        message.point.x = sample.elapsed_sec;
        message.point.y = sample.snr_db;
        message.point.z = 0.0;
        publisher->publish(message);
    }

    static const char * phase_name(const Phase phase)
    {
        switch (phase) {
            case Phase::FIRST_FORWARD: return "FIRST_FORWARD";
            case Phase::YAW: return "YAW";
            case Phase::SECOND_FORWARD: return "SECOND_FORWARD";
            case Phase::CLOCKWISE_YAW: return "CLOCKWISE_YAW";
            case Phase::SNR_FORWARD: return "SNR_FORWARD";
            case Phase::UTURN: return "UTURN";
            case Phase::RETURN_TO_MAX_SNR_TIME: return "RETURN_TO_MAX_SNR_TIME";
            case Phase::HANDOFF_NEUTRAL: return "HANDOFF_NEUTRAL";
            case Phase::COMPLETE: return "COMPLETE";
        }
        return "UNKNOWN";
    }

    double depth_hold_command(const rclcpp::Time & current_time)
    {
        const double error = target_depth_z_m_ - current_z_m_;
        double dt = 0.0;
        double error_derivative = 0.0;
        if (depth_pid_initialized_) {
            dt = std::clamp(
                (current_time - last_depth_control_time_).seconds(), 0.0, 0.2);
            if (dt > 1.0e-6) {
                error_derivative = (error - previous_depth_error_) / dt;
            }
        }
        last_depth_control_time_ = current_time;
        previous_depth_error_ = error;
        depth_pid_initialized_ = true;

        const double candidate_integral = std::clamp(
            depth_error_integral_ + error * dt,
            -depth_integral_limit_, depth_integral_limit_);
        const double candidate_heave = depth_bias_ - (
            depth_kp_ * error + depth_ki_ * candidate_integral +
            depth_kd_ * error_derivative);
        if (std::abs(candidate_heave) <= heave_limit_ ||
            (candidate_heave > heave_limit_ && error > 0.0) ||
            (candidate_heave < -heave_limit_ && error < 0.0))
        {
            depth_error_integral_ = candidate_integral;
        }

        return std::clamp(
            depth_bias_ - (
                depth_kp_ * error + depth_ki_ * depth_error_integral_ +
                depth_kd_ * error_derivative),
            -heave_limit_, heave_limit_);
    }

    std::uint16_t axis_pwm(const double value, const bool invert) const
    {
        const double axis = invert ? -value : value;
        const int pwm = static_cast<int>(std::llround(
            static_cast<double>(RC_NEUTRAL_PWM) +
            std::clamp(axis, -1.0, 1.0) * rc_pwm_span_));
        return static_cast<std::uint16_t>(std::clamp(pwm, PWM_MIN, PWM_MAX));
    }

    void publish_control_pwm(
        const std::uint16_t yaw_pwm, const std::uint16_t forward_pwm)
    {
        const std::uint16_t vertical_pwm = have_odometry_ ?
            axis_pwm(depth_hold_command(now()), true) : RC_NEUTRAL_PWM;
        publish_pwm(vertical_pwm, yaw_pwm, forward_pwm);
    }

    void publish_pwm(
        const std::uint16_t vertical_pwm, const std::uint16_t yaw_pwm,
        const std::uint16_t forward_pwm)
    {
        mavros_msgs::msg::OverrideRCIn message;
        message.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
        message.channels[VERTICAL_CHANNEL_INDEX] = vertical_pwm;
        message.channels[YAW_CHANNEL_INDEX] = yaw_pwm;
        message.channels[FORWARD_CHANNEL_INDEX] = forward_pwm;
        rc_publisher_->publish(message);
    }

    void publish_pwm(const std::uint16_t yaw_pwm, const std::uint16_t forward_pwm)
    {
        publish_pwm(RC_NEUTRAL_PWM, yaw_pwm, forward_pwm);
    }

    int first_forward_pwm_ = RC_NEUTRAL_PWM;
    double first_forward_duration_sec_ = 0.0;
    int yaw_pwm_ = RC_NEUTRAL_PWM;
    double yaw_duration_sec_ = 0.0;
    int second_forward_pwm_ = RC_NEUTRAL_PWM;
    double second_forward_duration_sec_ = 0.0;
    int clockwise_yaw_pwm_ = RC_NEUTRAL_PWM;
    double clockwise_yaw_duration_sec_ = 0.0;
    int snr_forward_pwm_ = RC_NEUTRAL_PWM;
    double snr_forward_duration_sec_ = 0.0;
    int uturn_yaw_pwm_ = RC_NEUTRAL_PWM;
    double uturn_yaw_duration_sec_ = 0.0;
    int return_forward_pwm_ = RC_NEUTRAL_PWM;
    double return_forward_duration_sec_ = 0.0;
    double snr_sample_period_sec_ = 0.1;
    int snr_median_window_size_ = 3;
    int global_max_region_window_size_ = 5;
    double vision_confirmation_lead_sec_ = 1.0;
    double target_depth_z_m_ = -8.0;
    double depth_kp_ = 0.8;
    double depth_ki_ = 0.15;
    double depth_kd_ = 0.0;
    double depth_bias_ = 0.0;
    double depth_integral_limit_ = 2.0;
    double heave_limit_ = 0.2;
    double rc_pwm_span_ = 400.0;
    double current_z_m_ = 0.0;
    double previous_depth_error_ = 0.0;
    double depth_error_integral_ = 0.0;

    std::vector<SnrSample> snr_map_;
    std::deque<SnrSample> snr_filter_window_;
    std::vector<SnrSample> filtered_snr_profile_;
    bool have_latest_snr_ = false;
    double latest_snr_db_ = 0.0;
    double next_snr_sample_elapsed_sec_ = 0.1;
    double max_snr_db_ = -std::numeric_limits<double>::infinity();
    double max_snr_elapsed_sec_ = 0.0;
    bool vision_search_requested_ = false;
    bool handoff_neutral_sent_ = false;
    bool have_odometry_ = false;
    bool depth_pid_initialized_ = false;

    Phase phase_ = Phase::FIRST_FORWARD;
    rclcpp::Time phase_start_time_;
    rclcpp::Time last_depth_control_time_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_publisher_;
    rclcpp::Subscription<audio_common_msgs::msg::Float64Stamped>::SharedPtr
        snr_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        target_confirmed_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        vision_search_request_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        vision_control_granted_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr timeout_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        raw_snr_map_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        filtered_snr_map_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        selected_snr_target_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hydrophone_ctrl

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<hydrophone_ctrl::HaredcodedPwmNode>());
    rclcpp::shutdown();
    return 0;
}
