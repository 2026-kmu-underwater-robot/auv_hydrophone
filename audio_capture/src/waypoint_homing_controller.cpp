#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace audio_capture
{
// 초기 pose를 arena (0,0,+X)로 고정하고 모든 scan/homing 이동을 절대 odometry
// waypoint로 수행한다. Depth, 도착 판정, recovery는 이 버전에 포함하지 않는다.
class WaypointHomingControllerNode : public rclcpp::Node
{
public:
    explicit WaypointHomingControllerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("waypoint_homing_controller", options)
    {
        const auto odometry_topic = declare_parameter<std::string>(
            "odometry_topic", "/odometry/filtered");
        const auto region_gradient_topic = declare_parameter<std::string>(
            "region_gradient_topic", "/homing/region_gradient");
        const auto rolling_gradient_topic = declare_parameter<std::string>(
            "rolling_gradient_topic", "/homing/rolling_gradient");
        const auto homing_direction_topic = declare_parameter<std::string>(
            "homing_direction_topic", "/homing/homing_direction");
        const auto snr_trend_topic = declare_parameter<std::string>(
            "snr_trend_topic", "/homing/snr_trend");
        const auto state_topic = declare_parameter<std::string>(
            "state_topic", "/homing/control_state");
        const auto waypoint_topic = declare_parameter<std::string>(
            "waypoint_topic", "/homing/current_waypoint");
        const auto scan_center_topic = declare_parameter<std::string>(
            "scan_center_topic", "/homing/scan_center");
        const auto vision_search_request_topic = declare_parameter<std::string>(
            "vision_search_request_topic", "/homing/vision_search_active");
        const auto target_confirmed_topic = declare_parameter<std::string>(
            "target_confirmed_topic", "/vision/target_confirmed");
        const auto rc_override_topic = declare_parameter<std::string>(
            "rc_override_topic", "/mavros/rc/override");

        arena_length_m_ = std::max(
            0.1, declare_parameter<double>("arena_length_m", 15.0));
        arena_width_m_ = std::max(
            0.1, declare_parameter<double>("arena_width_m", 16.0));
        arena_offset_x_m_ = declare_parameter<double>("arena_offset_x_m", 0.0);
        arena_offset_y_m_ = declare_parameter<double>("arena_offset_y_m", 0.0);
        if (!std::isfinite(arena_offset_x_m_) || !std::isfinite(arena_offset_y_m_)) {
            throw std::invalid_argument("arena offsets must be finite");
        }
        arena_start_corner_ = declare_parameter<std::string>(
            "arena_start_corner", "bottom_left");
        if (arena_start_corner_ != "bottom_left" &&
            arena_start_corner_ != "bottom_right")
        {
            throw std::invalid_argument(
                "arena_start_corner must be bottom_left or bottom_right");
        }
        arena_safety_margin_m_ = std::max(
            0.0, declare_parameter<double>("arena_safety_margin_m", 0.5));
        initial_scan_radius_m_ = std::max(
            0.01, declare_parameter<double>("initial_scan_radius_m", 1.5));
        rescan_radius_m_ = std::max(
            0.01, declare_parameter<double>("rescan_radius_m", 0.7));
        if (2.0 * (arena_safety_margin_m_ +
            std::max(initial_scan_radius_m_, rescan_radius_m_)) >
            std::min(arena_length_m_, arena_width_m_))
        {
            throw std::invalid_argument(
                "arena is too small for scan radius and arena_safety_margin_m");
        }

        region_scan_waypoint_count_ = static_cast<std::size_t>(std::max<std::int64_t>(
            3, declare_parameter<std::int64_t>("region_scan_waypoint_count", 8)));
        homing_waypoint_step_m_ = std::max(
            0.05, declare_parameter<double>("homing_waypoint_step_m", 0.8));
        homing_zigzag_offset_m_ = std::max(
            0.0, declare_parameter<double>("homing_zigzag_offset_m", 0.2));
        rolling_gradient_alpha_ = std::clamp(
            declare_parameter<double>("rolling_gradient_alpha", 0.15), 0.0, 1.0);
        waypoint_reach_tolerance_m_ = std::max(
            0.01, declare_parameter<double>("waypoint_reach_tolerance_m", 0.15));
        waypoint_dwell_s_ = std::max(
            0.0, declare_parameter<double>("waypoint_dwell_s", 0.1));
        slope_decrease_threshold_db_per_m2_ = std::max(
            0.0, declare_parameter<double>(
                "slope_decrease_threshold_db_per_m2", 1.0));
        slope_decrease_limit_ = static_cast<std::size_t>(std::max<std::int64_t>(
            1, declare_parameter<std::int64_t>("slope_decrease_limit", 5)));
        vision_near_zone_width_m_ = std::clamp(
            declare_parameter<double>("vision_near_zone_width_m", 2.0),
            0.0, arena_width_m_);
        vision_handoff_enabled_ = declare_parameter<bool>(
            "vision_handoff_enabled", true);

        rate_hz_ = std::clamp(
            declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);
        odometry_timeout_s_ = std::max(
            0.05, declare_parameter<double>("odometry_timeout_s", 0.5));
        forward_gain_ = std::max(
            0.0, declare_parameter<double>("forward_gain", 0.8));
        forward_limit_ = std::clamp(
            declare_parameter<double>("forward_limit", 0.5), 0.0, 1.0);
        yaw_gain_ = std::max(
            0.0, declare_parameter<double>("yaw_gain", 1.15));
        yaw_limit_ = std::clamp(
            declare_parameter<double>("yaw_limit", 0.72), 0.0, 1.0);
        move_heading_tolerance_rad_ = std::clamp(
            declare_parameter<double>("move_heading_tolerance_rad", 0.35), 0.01, PI);
        vision_heading_tolerance_rad_ = std::clamp(
            declare_parameter<double>("vision_heading_tolerance_rad", 0.12), 0.01, PI);
        rc_pwm_span_ = std::clamp(
            declare_parameter<double>("rc_pwm_span", 400.0), 50.0, 700.0);
        invert_rc_yaw_ = declare_parameter<bool>("invert_rc_yaw", true);

        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 30,
            std::bind(&WaypointHomingControllerNode::odometry_callback, this,
                std::placeholders::_1));
        region_gradient_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            region_gradient_topic, 10,
            std::bind(&WaypointHomingControllerNode::region_gradient_callback, this,
                std::placeholders::_1));
        rolling_gradient_sub_ =
            create_subscription<geometry_msgs::msg::Vector3Stamped>(
                rolling_gradient_topic, 10,
                std::bind(&WaypointHomingControllerNode::rolling_gradient_callback,
                    this, std::placeholders::_1));
        snr_trend_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            snr_trend_topic, 10,
            std::bind(&WaypointHomingControllerNode::snr_trend_callback, this,
                std::placeholders::_1));
        target_confirmed_sub_ = create_subscription<std_msgs::msg::Bool>(
            target_confirmed_topic, 10,
            std::bind(&WaypointHomingControllerNode::target_confirmed_callback, this,
                std::placeholders::_1));
        state_pub_ = create_publisher<std_msgs::msg::String>(
            state_topic, rclcpp::QoS(1).reliable().transient_local());
        waypoint_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            waypoint_topic, rclcpp::QoS(1).reliable().transient_local());
        scan_center_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            scan_center_topic, rclcpp::QoS(1).reliable().transient_local());
        vision_search_request_pub_ = create_publisher<std_msgs::msg::Bool>(
            vision_search_request_topic,
            rclcpp::QoS(1).reliable().transient_local());
        homing_direction_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>(
                homing_direction_topic,
                rclcpp::QoS(1).reliable().transient_local());
        rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(
            rc_override_topic, 10);

        publish_state();
        publish_vision_search_request(false);
        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&WaypointHomingControllerNode::control_loop, this));

        const ArenaBounds bounds = arena_bounds(0.0);
        RCLCPP_INFO(
            get_logger(),
            "Waypoint controller ready: arena x=[%.3f, %.3f] y=[%.3f, %.3f]",
            bounds.x_min, bounds.x_max, bounds.y_min, bounds.y_max);
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr std::uint16_t RC_NEUTRAL = 1500;
    static constexpr std::size_t VERTICAL_CHANNEL_INDEX = 2;
    static constexpr std::size_t YAW_CHANNEL_INDEX = 3;
    static constexpr std::size_t FORWARD_CHANNEL_INDEX = 4;
    static constexpr std::size_t LATERAL_CHANNEL_INDEX = 5;
    static constexpr std::size_t PRIMARY_CHANNEL_COUNT = 8;

    enum class State
    {
        MOVE_TO_SCAN_CENTER,    //원형 탐색 중심 위치로 이동 상태
        REGION_SCAN,    //원형 탐색 상태
        REGION_HOMING,    //원형 탐색 결과 그래디언트 매칭 상태
        VISION_SEARCH,    //비전 제어기가 부표를 탐색하도록 제어권을 인계한 상태
        VISION_APPROACH    //음향 제어를 끝내고 비전 제어기로 인계한 상태
    };

    enum class HomingTrendState
    {
        UNKNOWN,
        TRACKING,
        DECREASING
    };

    struct ArenaBounds
    {
        double x_min = 0.0;
        double x_max = 0.0;
        double y_min = 0.0;
        double y_max = 0.0;
    };

    struct Command
    {
        double forward = 0.0;
        double yaw = 0.0;
    };

    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // 현재 위치와 yaw를 업데이트한다.
        const Eigen::Vector2d position(
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        const double yaw = yaw_from_quaternion(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        if (!position.allFinite() || !std::isfinite(yaw)) { // 위치와 yaw가 유효하지 않으면 무시한다.
            return;
        }
        current_position_ = position; // 현재 위치를 업데이트한다.
        current_yaw_rad_ = yaw; // 현재 yaw를 업데이트한다.
        odometry_frame_ = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id; // 오돔 메세지가 어느 좌표계 기준인지 업데이트한다 (빈 문자열이면 odom 기준).
        last_odometry_receive_time_ = now(); // 마지막 오도메트리 수신 시간을 업데이트한다.
        have_odometry_ = true; // 오도메트리 수신 여부를 업데이트한다.

        if (!mission_started_) { // 최초 odometry 수신 시 미션을 시작한다.
            mission_started_ = true;
            start_new_region_scan(arena_center(), true); // 최초 탐색은 실험장 중앙에서 시작.
        }
    }

    void region_gradient_callback(
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        if (state_ != State::REGION_SCAN) {
            return;
        }
        const Eigen::Vector2d gradient(msg->vector.x, msg->vector.y);
        const double magnitude = gradient.norm();
        const bool valid = gradient.allFinite() && std::isfinite(magnitude) &&
            magnitude > 1.0e-6;

        region_result_received_ = true;
        region_gradient_valid_ = valid;
        if (valid) {
            region_gradient_ = gradient / magnitude;
        }
        RCLCPP_DEBUG(
            get_logger(), "REGION_SCAN gradient: valid=%s raw=(%.3f, %.3f) "
            "G_ref=(%.3f, %.3f)",
            valid ? "true" : "false", gradient.x(), gradient.y(),
            region_gradient_.x(), region_gradient_.y());
    }

    void snr_trend_callback(
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        if (state_ != State::REGION_HOMING || new_scan_requested_) {
            return;
        }
        const double slope = msg->vector.x;
        const double slope_change = msg->vector.y;
        if (!std::isfinite(slope) || !std::isfinite(slope_change)) {
            return;
        }

        // Vision 탐색 요청 이후의 제어권 전환은 target_confirmed만으로 결정한다.
        if (vision_search_requested_) {
            return;
        }

        if (slope_change < -slope_decrease_threshold_db_per_m2_) {
            ++slope_decrease_count_;
            log_homing_trend_change(
                HomingTrendState::DECREASING, slope, slope_change,
                slope_decrease_count_, slope_decrease_limit_);
            if (slope_decrease_count_ >= slope_decrease_limit_) {
                new_scan_requested_ = true;
                RCLCPP_WARN(
                    get_logger(),
                    "[RESCAN] reason=decreasing_slope slope=%.3f dB/m "
                    "change=%.3f dB/m^2 consecutive=%zu",
                    slope, slope_change, slope_decrease_count_);
            }
            return;
        }

        slope_decrease_count_ = 0;
        log_homing_trend_change(
            HomingTrendState::TRACKING, slope, slope_change, 0, 0);
    }

    void rolling_gradient_callback(
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        if (state_ != State::REGION_HOMING || vision_search_requested_) {
            return;
        }
        const Eigen::Vector2d rolling(msg->vector.x, msg->vector.y);
        const double magnitude = rolling.norm();
        if (!rolling.allFinite() || magnitude <= 1.0e-6) {
            return;
        }
        const Eigen::Vector2d blended =
            (1.0 - rolling_gradient_alpha_) * homing_direction_ +
            rolling_gradient_alpha_ * rolling / magnitude;
        if (blended.norm() > 1.0e-6) {
            homing_direction_ = blended.normalized();
            publish_homing_direction();
        }
    }

    void target_confirmed_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if (!msg->data || !vision_handoff_enabled_ ||
            !vision_search_requested_ ||
            (state_ != State::REGION_HOMING && state_ != State::VISION_SEARCH))
        {
            return;
        }
        publish_rc(Command{});
        transition_to(State::VISION_APPROACH);
        RCLCPP_INFO(get_logger(), "[VISION] target confirmed; control handed off");
    }

    void log_homing_trend_change(
        const HomingTrendState next_state,
        const double slope,
        const double slope_change,
        const std::size_t count,
        const std::size_t limit)
    {
        if (homing_trend_state_ == next_state) {
            return;
        }
        homing_trend_state_ = next_state;
        if (next_state == HomingTrendState::TRACKING) {
            RCLCPP_INFO(
                get_logger(), "[HOMING] TRACKING slope=%.3f dB/m change=%.3f dB/m^2",
                slope, slope_change);
        } else if (next_state == HomingTrendState::DECREASING) {
            RCLCPP_WARN(
                get_logger(),
                "[HOMING] DECREASING slope=%.3f dB/m change=%.3f dB/m^2 (%zu/%zu)",
                slope, slope_change, count, limit);
        }
    }

    bool inside_vision_zone() const
    {
        if (vision_near_zone_width_m_ <= 0.0) {
            return false;
        }
        const ArenaBounds bounds = arena_bounds(arena_safety_margin_m_);
        const double width = std::min(
            vision_near_zone_width_m_, bounds.y_max - bounds.y_min);
        if (arena_start_corner_ == "bottom_left") {
            return current_position_.y() <=
                bounds.y_min + width;
        }
        return current_position_.y() >=
            bounds.y_max - width;
    }

    void request_vision_search()
    {
        if (vision_search_requested_) {
            return;
        }
        vision_search_requested_ = true;
        publish_vision_search_request(true);
        slope_decrease_count_ = 0;
        publish_homing_direction();
        RCLCPP_INFO(
            get_logger(),
            "[VISION] search requested at position=(%.2f, %.2f) m",
            current_position_.x(), current_position_.y());
        if (state_ == State::REGION_HOMING) {
            if (!make_next_homing_waypoint()) {
                begin_vision_search();
                return;
            }
            set_current_waypoint(waypoints_.front());
        }
    }

    void begin_vision_search()
    {
        request_vision_search();
        publish_rc(Command{});
        transition_to(State::VISION_SEARCH);
        RCLCPP_INFO(
            get_logger(),
            "[VISION] arena boundary reached inside near zone; "
            "acoustic control handed off for visual search");
    }

    void control_loop()
    {
        const rclcpp::Time current_time = now(); // 현재 시간을 가져온다.
        if (state_ == State::VISION_SEARCH || state_ == State::VISION_APPROACH) {
            log_controller_status();
            return;
        }
        if (!mission_started_ || !odometry_is_fresh(current_time) ||
            !have_current_waypoint_)
        {
            publish_rc(Command{}); // 미션이 시작되지 않았거나 오도메트리 수신이 없거나 waypoint가 설정되지 않았으면 명령을 보내지 않는다.
            return;
        }
        log_controller_status();

        if (state_ == State::REGION_HOMING && vision_handoff_enabled_ &&
            inside_vision_zone())
        {
            request_vision_search();
        }

        switch (state_) {
            case State::MOVE_TO_SCAN_CENTER:    //원형 탐색 중심 위치로 이동하기 위한 준비 상태
                if (follow_waypoint(current_time)) { 
                    begin_region_scan(); // 원형 탐색 준비
                }
                return;

            case State::REGION_SCAN:    //원형 탐색 상태
                if (!follow_waypoint(current_time)) { // 아직 도착하지 않았으면 계속 이동
                    return;
                }
                if (waypoint_index_ < waypoints_.size()) {
                    RCLCPP_INFO(
                        get_logger(), "[SCAN] waypoint %zu/%zu complete",
                        waypoint_index_ + 1, waypoints_.size());
                }
                if (++waypoint_index_ < waypoints_.size()) { // waypoint 인덱스를 증가시키고 다음 waypoint로 이동
                    set_current_waypoint(waypoints_[waypoint_index_]); 
                    return;
                }
                publish_rc(Command{}); //원형 탐색 상태에서 마지막 waypoint까지 돌고나면 일단 중립 명령을 보낸다.
                if (!region_result_received_) { //원형 탐색 결과 수신 플래그가 꺼져있으면 결과가 수신되지 않았으므로 대기.
                    return;
                }
                if (!region_gradient_valid_) { //원형 탐색 결과 그래디언트 유효성 판정 플래그가 꺼져있으면 그래디언트 유효성 판정 실패이므로 새로운 원형 탐색을 요청.
                    RCLCPP_WARN(
                        get_logger(), "[RESCAN] reason=invalid_region_gradient");
                    start_new_region_scan(current_position_); // 현재 위치 기준으로 새로운 원형 탐색을 요청.
                    return;
                }
                begin_region_homing(); //원형 탐색 결과 그래디언트 유효성 판정 성공이므로 원형 탐색 결과 그래디언트 매칭 상태로 전이.
                return;

            case State::REGION_HOMING:
                if (new_scan_requested_) {
                    start_new_region_scan(current_position_);
                    return;
                }
                if (!follow_waypoint(current_time)) {
                    return;
                }
                if (!make_next_homing_waypoint()) {
                    handle_homing_boundary();
                    return;
                }
                set_current_waypoint(waypoints_.front());
                return;

            case State::VISION_SEARCH:
            case State::VISION_APPROACH:
                // 진입 시 한 번 중립 명령을 보낸 뒤 비전 제어기의 RC 출력을 방해하지 않는다.
                return;
        }
    }

    void start_new_region_scan(const Eigen::Vector2d & requested_center, const bool force = false)
    {
        if (vision_search_requested_) {
            publish_vision_search_request(false);
            vision_search_requested_ = false;
        }
        active_scan_radius_m_ = first_region_scan_ ?
            initial_scan_radius_m_ : rescan_radius_m_;
        first_region_scan_ = false;
        scan_center_ = adjusted_scan_center(requested_center, active_scan_radius_m_);
        publish_scan_center();
        region_result_received_ = false;
        region_gradient_valid_ = false;
        new_scan_requested_ = false;
        slope_decrease_count_ = 0;
        homing_trend_state_ = HomingTrendState::UNKNOWN;
        region_gradient_ = Eigen::Vector2d::UnitX();
        waypoints_.clear();
        transition_to(State::MOVE_TO_SCAN_CENTER, force); // 원형 탐색 중심 위치로 이동 상태로 전이.
        set_current_waypoint(scan_center_); // 원형 탐색 중심 위치를 waypoint로 설정.
    }

    void begin_region_scan() // 원형 탐색 준비를 위한  waypoint 설정 및 상태 전이
    {
        waypoints_.clear();//waypoint 벡터 초기화
        waypoints_.reserve(region_scan_waypoint_count_);//waypoint 벡터 용량 예약
        for (std::size_t i = 0; i < region_scan_waypoint_count_; ++i) { //기본 8개 waypoint에 대해서
            const double angle = 2.0 * PI * static_cast<double>(i) / //원을 waypoint 개수로 나누어 각 각도를 계산한다.
                static_cast<double>(region_scan_waypoint_count_);
            const Eigen::Vector2d waypoint = scan_center_ +
                active_scan_radius_m_ * Eigen::Vector2d(std::cos(angle), std::sin(angle));//원 중심에서 탐색 반경만큼 떨어진 점 계산.
            if (!waypoint_is_safe(waypoint)) {//원형 탐색 경로가 실험장 안쪽에 있는지 판정
                throw std::logic_error("adjusted region scan circle left arena safety bounds");//원형 탐색 경로가 실험장 안쪽에 있지 않으면 예외 발생
            }
            waypoints_.push_back(waypoint); //waypoint 벡터에 추가.
        }
        waypoint_index_ = 0; //0번 waypoint부터 시작.
        region_result_received_ = false; //원형 탐색 결과 수신 플래그를 초기화.
        region_gradient_valid_ = false;//원형 탐색 결과 그래디언트 유효성 판정 플래그를 초기화.
        transition_to(State::REGION_SCAN);//원형 탐색 상태로 전이.
        set_current_waypoint(waypoints_.front());//원형 탐색 중심 위치를 waypoint로 설정.
    }

    void begin_region_homing() // 원형 탐색 결과 그래디언트 매칭 상태로 전이
    {
        new_scan_requested_ = false;
        slope_decrease_count_ = 0;
        homing_trend_state_ = HomingTrendState::UNKNOWN;
        homing_direction_ = region_gradient_;
        zigzag_sign_ = 1.0;
        transition_to(State::REGION_HOMING);
        publish_homing_direction();
        RCLCPP_INFO(
            get_logger(), "[SCAN] complete G_ref=(%.3f, %.3f)",
            region_gradient_.x(), region_gradient_.y());
        if (!make_next_homing_waypoint()) {
            handle_homing_boundary();
            return;
        }
        set_current_waypoint(waypoints_.front()); // Region Gradient 방향의 첫 waypoint를 설정.
    }

    bool make_next_homing_waypoint() // 현재 위치에서 Region Gradient 방향의 waypoint를 생성한다.
    {
        Eigen::Vector2d delta = homing_waypoint_step_m_ * homing_direction_;
        if (!vision_search_requested_) {
            const Eigen::Vector2d normal(-homing_direction_.y(), homing_direction_.x());
            delta += zigzag_sign_ * homing_zigzag_offset_m_ * normal;
        }
        Eigen::Vector2d waypoint = current_position_ + delta;
        const ArenaBounds bounds = arena_bounds(arena_safety_margin_m_);
        waypoint.x() = std::clamp(waypoint.x(), bounds.x_min, bounds.x_max);
        waypoint.y() = std::clamp(waypoint.y(), bounds.y_min, bounds.y_max);
        if ((waypoint - current_position_).norm() <= waypoint_reach_tolerance_m_) {
            return false;
        }
        waypoints_.assign(1, waypoint);
        if (!vision_search_requested_) {
            zigzag_sign_ = -zigzag_sign_;
        }
        return true;
    }

    void handle_homing_boundary()
    {
        if (vision_handoff_enabled_ && inside_vision_zone()) {
            begin_vision_search();
            return;
        }
        RCLCPP_WARN(get_logger(), "[RESCAN] reason=arena_boundary");
        start_new_region_scan(current_position_);
    }

    bool follow_waypoint(const rclcpp::Time & current_time) // waypoint까지 이동하고 도착했는지에 대한 플래그.
    {
        const Eigen::Vector2d delta = current_waypoint_ - current_position_; // 현재 위치와 waypoint 사이의 벡터
        const double distance = delta.norm(); // 현재 위치와 waypoint 사이의 거리
        if (distance <= waypoint_reach_tolerance_m_) { // 만약 현재 위치와 waypoint 사이의 거리가 허용 오차 이내면 
            publish_rc(Command{}); // RC 중립 명령.
            if (!waypoint_dwell_active_) { // waypoint 도착 후 일정 시간 동안 대기하는 플래그가 꺼져있으면 켜고 시간을 업데이트한다.
                waypoint_dwell_active_ = true;
                waypoint_arrival_time_ = current_time; // waypoint 도착 시각 저장
                RCLCPP_DEBUG(
                    get_logger(), "%s waypoint arrival radius entered: distance=%.3f m",
                    state_name(state_), distance);
            }
            return (current_time - waypoint_arrival_time_).seconds() >= waypoint_dwell_s_; // waypoint 도착 후 일정 시간 이상 대기했는지를 보고 도착 유무 판단.
        }
        if (waypoint_dwell_active_) {
            RCLCPP_DEBUG(
                get_logger(), "%s waypoint arrival radius left; dwell reset: distance=%.3f m",
                state_name(state_), distance);
        }
        waypoint_dwell_active_ = false; // 아직 안도착한 경우 이므로 대기 플래그를 끈다.
        const double desired_yaw = std::atan2(delta.y(), delta.x()); // 현재 위치와 waypoint 사이의 벡터를 이용하여 원하는 yaw 값을 계산한다.
        const double yaw_error = wrap_pi(desired_yaw - current_yaw_rad_); // 현재 yaw와 원하는 yaw 사이의 오차.
        Command command; // 명령 구조체 선언.
        command.yaw = std::clamp(yaw_gain_ * yaw_error, -yaw_limit_, yaw_limit_); // yaw 명령 값을 클램프 함수를 이용하여 제한한다.
        const double heading_tolerance = vision_search_requested_ ?
            vision_heading_tolerance_rad_ : move_heading_tolerance_rad_;
        if (std::abs(yaw_error) <= heading_tolerance) { // yaw 일정 오차 이상 정렬되면 그때부터 전진.
            command.forward = std::clamp(forward_gain_ * distance, 0.0, forward_limit_); // 전진 명령 값을 클램프 함수를 이용하여 제한한다.
        }
        publish_rc(command); // 명령을 보낸다.
        return false; // 아직 도착하지 않았으므로 false를 반환.
    }

    void log_controller_status()
    {
        if (state_ == State::VISION_SEARCH || state_ == State::VISION_APPROACH) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[CONTROL] state=%s acoustic_control=inactive",
                state_name(state_));
            return;
        }
        const double distance = (current_waypoint_ - current_position_).norm();
        const std::size_t waypoint_count =
            state_ == State::REGION_SCAN ? waypoints_.size() : 1;
        const std::size_t current_index = state_ == State::REGION_SCAN ?
            std::min(waypoint_index_ + 1, waypoint_count) : 1;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[CONTROL] state=%s waypoint=%zu/%zu distance=%.2f m",
            state_name(state_), current_index, waypoint_count, distance);
    }

    Eigen::Vector2d adjusted_scan_center(
        const Eigen::Vector2d & position, const double radius) const // 원형 탐색 중심 위치를 실험장 안쪽으로 조정하는 함수
    {
        Eigen::Vector2d center = position;
        const double inset = arena_safety_margin_m_ + radius; // 중심이 벽에서 최소 이만큼 떨어져야 반경 원이 여유공간을 안 뚫음
        const ArenaBounds bounds = arena_bounds(inset);
        center.x() = std::clamp(center.x(), bounds.x_min, bounds.x_max);
        center.y() = std::clamp(center.y(), bounds.y_min, bounds.y_max);
        return center; //보정된 중심 위치를 반환.
    }

    Eigen::Vector2d arena_center() const
    {
        const ArenaBounds bounds = arena_bounds(0.0);
        return {
            0.5 * (bounds.x_min + bounds.x_max),
            0.5 * (bounds.y_min + bounds.y_max)};
    }


    // 원형 탐색 경로가 실험장 안쪽에 있는지 판정하는 함수
    bool waypoint_is_safe(const Eigen::Vector2d & waypoint) const
    {
        const ArenaBounds bounds = arena_bounds(arena_safety_margin_m_);
        return waypoint.x() >= bounds.x_min && waypoint.x() <= bounds.x_max &&
            waypoint.y() >= bounds.y_min && waypoint.y() <= bounds.y_max;
    }

    ArenaBounds arena_bounds(const double inset) const
    {
        ArenaBounds bounds;
        bounds.x_min = arena_offset_x_m_ + inset;
        bounds.x_max = arena_offset_x_m_ + arena_length_m_ - inset;
        if (arena_start_corner_ == "bottom_left") {
            bounds.y_min = arena_offset_y_m_ - arena_width_m_ + inset;
            bounds.y_max = arena_offset_y_m_ - inset;
        } else {
            bounds.y_min = arena_offset_y_m_ + inset;
            bounds.y_max = arena_offset_y_m_ + arena_width_m_ - inset;
        }
        return bounds;
    }

    void set_current_waypoint(const Eigen::Vector2d & waypoint)
    {
        current_waypoint_ = waypoint;
        have_current_waypoint_ = true;
        waypoint_dwell_active_ = false;
        current_waypoint_set_time_ = now();
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = current_waypoint_set_time_;
        msg.header.frame_id = odometry_frame_;
        msg.point.x = waypoint.x();
        msg.point.y = waypoint.y();
        waypoint_pub_->publish(msg);
    }

    void publish_scan_center()
    {
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = odometry_frame_;
        msg.point.x = scan_center_.x();
        msg.point.y = scan_center_.y();
        scan_center_pub_->publish(msg);
    }

    void transition_to(const State next_state, const bool force = false)
    {
        if (!force && state_ == next_state) {
            return;
        }
        const State previous_state = state_;
        state_ = next_state;
        publish_state();
        RCLCPP_INFO(
            get_logger(), "[STATE] %s -> %s",
            state_name(previous_state), state_name(state_));
    }

    void publish_state()
    {
        std_msgs::msg::String msg;
        msg.data = state_name(state_);
        state_pub_->publish(msg);
    }

    void publish_vision_search_request(const bool active)
    {
        std_msgs::msg::Bool msg;
        msg.data = active;
        vision_search_request_pub_->publish(msg);
    }

    void publish_homing_direction()
    {
        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = odometry_frame_;
        msg.vector.x = homing_direction_.x();
        msg.vector.y = homing_direction_.y();
        homing_direction_pub_->publish(msg);
    }

    void publish_rc(const Command & command)
    {
        mavros_msgs::msg::OverrideRCIn msg;
        msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
        for (std::size_t i = 0; i < PRIMARY_CHANNEL_COUNT; ++i) {
            msg.channels[i] = RC_NEUTRAL;
        }
        msg.channels[VERTICAL_CHANNEL_INDEX] = RC_NEUTRAL;
        msg.channels[YAW_CHANNEL_INDEX] = axis_pwm(command.yaw, invert_rc_yaw_);
        msg.channels[FORWARD_CHANNEL_INDEX] = axis_pwm(command.forward, false);
        msg.channels[LATERAL_CHANNEL_INDEX] = RC_NEUTRAL;
        rc_pub_->publish(msg);
    }

    std::uint16_t axis_pwm(const double value, const bool invert) const
    {
        const double axis = invert ? -value : value;
        const int pwm = static_cast<int>(std::llround(
            static_cast<double>(RC_NEUTRAL) +
            std::clamp(axis, -1.0, 1.0) * rc_pwm_span_));
        return static_cast<std::uint16_t>(std::clamp(pwm, 1100, 1900));
    }

    bool odometry_is_fresh(const rclcpp::Time & current_time) const
    {
        return have_odometry_ &&
            (current_time - last_odometry_receive_time_).seconds() <= odometry_timeout_s_;
    }

    static double wrap_pi(const double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    static double yaw_from_quaternion(
        const double w, const double x, const double y, const double z)
    {
        return std::atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z));
    }

    static const char * state_name(const State state)
    {
        switch (state) {
            case State::MOVE_TO_SCAN_CENTER:
                return "MOVE_TO_SCAN_CENTER";
            case State::REGION_SCAN:
                return "REGION_SCAN";
            case State::REGION_HOMING:
                return "REGION_HOMING";
            case State::VISION_SEARCH:
                return "VISION_SEARCH";
            case State::VISION_APPROACH:
                return "VISION_APPROACH";
        }
        return "MOVE_TO_SCAN_CENTER";
    }

    double arena_length_m_ = 15.0;
    double arena_width_m_ = 16.0;
    double arena_offset_x_m_ = 0.0;
    double arena_offset_y_m_ = 0.0;
    double arena_safety_margin_m_ = 0.5;
    double initial_scan_radius_m_ = 1.5;
    double rescan_radius_m_ = 0.7;
    double active_scan_radius_m_ = 1.5;
    double homing_waypoint_step_m_ = 0.8;
    double homing_zigzag_offset_m_ = 0.2;
    double rolling_gradient_alpha_ = 0.15;
    double waypoint_reach_tolerance_m_ = 0.15;
    double waypoint_dwell_s_ = 0.1;
    double slope_decrease_threshold_db_per_m2_ = 1.0;
    double vision_near_zone_width_m_ = 2.0;
    double rate_hz_ = 30.0;
    double odometry_timeout_s_ = 0.5;
    double forward_gain_ = 0.8;
    double forward_limit_ = 0.5;
    double yaw_gain_ = 1.15;
    double yaw_limit_ = 0.72;
    double move_heading_tolerance_rad_ = 0.35;
    double vision_heading_tolerance_rad_ = 0.12;
    double rc_pwm_span_ = 400.0;
    std::size_t region_scan_waypoint_count_ = 8;
    std::size_t slope_decrease_limit_ = 5;
    std::size_t waypoint_index_ = 0;
    std::size_t slope_decrease_count_ = 0;
    double zigzag_sign_ = 1.0;
    std::string arena_start_corner_ = "bottom_left";
    std::string odometry_frame_ = "odom";
    bool invert_rc_yaw_ = true;
    bool vision_handoff_enabled_ = true;
    bool have_odometry_ = false;
    bool mission_started_ = false;
    bool have_current_waypoint_ = false;
    bool waypoint_dwell_active_ = false;
    bool region_result_received_ = false;
    bool region_gradient_valid_ = false;
    bool new_scan_requested_ = false;
    bool vision_search_requested_ = false;
    bool first_region_scan_ = true;
    State state_ = State::MOVE_TO_SCAN_CENTER;
    HomingTrendState homing_trend_state_ = HomingTrendState::UNKNOWN;
    rclcpp::Time last_odometry_receive_time_;
    rclcpp::Time waypoint_arrival_time_;
    rclcpp::Time current_waypoint_set_time_;
    double current_yaw_rad_ = 0.0;
    Eigen::Vector2d current_position_{0.0, 0.0};
    Eigen::Vector2d scan_center_{0.0, 0.0};
    Eigen::Vector2d current_waypoint_{0.0, 0.0};
    Eigen::Vector2d region_gradient_{1.0, 0.0};
    Eigen::Vector2d homing_direction_{1.0, 0.0};
    std::vector<Eigen::Vector2d> waypoints_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        region_gradient_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        rolling_gradient_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr snr_trend_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_confirmed_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr scan_center_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_search_request_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        homing_direction_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::WaypointHomingControllerNode)
