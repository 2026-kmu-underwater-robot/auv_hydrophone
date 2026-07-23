#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace audio_capture
{
// odometry에서 경로를 생성하고 arena 좌표 waypoint로 변환해 외부 추종기에 전달한다.
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
        const auto state_topic = declare_parameter<std::string>(
            "state_topic", "/homing/control_state");
        const auto waypoint_topic = declare_parameter<std::string>(
            "waypoint_topic", "/waypoint");
        arena_frame_id_ = declare_parameter<std::string>(
            "arena_frame_id", "arena");
        const auto scan_center_topic = declare_parameter<std::string>(
            "scan_center_topic", "/homing/scan_center");
        const auto vision_search_request_topic = declare_parameter<std::string>(
            "vision_search_request_topic", "/homing/vision_search_active");
        const auto target_confirmed_topic = declare_parameter<std::string>(
            "target_confirmed_topic", "/vision/target_confirmed");
        // [ACOUSTIC-VISION HANDSHAKE] Vision은 이 승인 후에만 제어를 시작한다.
        const auto vision_control_granted_topic = declare_parameter<std::string>(
            "vision_control_granted_topic", "/homing/vision_control_granted");
        const auto emergency_stop_topic = declare_parameter<std::string>(
            "emergency_stop_topic", "/mission/emergency_stop");
        const bool enable_keyboard_emergency_stop = declare_parameter<bool>(
            "enable_keyboard_emergency_stop", true);
        emergency_stop_key_ = declare_parameter<std::string>(
            "emergency_stop_key", "s");

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

        homing_waypoint_step_m_ = std::max(
            0.05, declare_parameter<double>("homing_waypoint_step_m", 0.8));
        homing_zigzag_offset_m_ = std::max(
            0.0, declare_parameter<double>("homing_zigzag_offset_m", 0.2));
        rolling_gradient_alpha_ = std::clamp(
            declare_parameter<double>("rolling_gradient_alpha", 0.15), 0.0, 1.0);
        rolling_gradient_conflict_angle_rad_ = std::clamp(
            declare_parameter<double>(
                "rolling_gradient_conflict_angle_rad", PI / 3.0),
            0.0, PI);
        rolling_gradient_conflict_limit_ =
            static_cast<std::size_t>(std::max<std::int64_t>(
                1,
                declare_parameter<std::int64_t>(
                    "rolling_gradient_conflict_limit", 3)));
        waypoint_reach_tolerance_m_ = std::max(
            0.01, declare_parameter<double>("waypoint_reach_tolerance_m", 0.15));
        scan_waypoint_lookahead_rad_ = std::clamp(
            declare_parameter<double>("scan_waypoint_lookahead_rad", 0.35),
            0.01, PI / 2.0);
        vision_near_zone_width_m_ = std::clamp(
            declare_parameter<double>("vision_near_zone_width_m", 2.0),
            0.0, arena_width_m_);
        vision_handoff_enabled_ = declare_parameter<bool>(
            "vision_handoff_enabled", true);
        target_depth_z_m_ = declare_parameter<double>("target_depth_z_m", -0.65);
        if (!std::isfinite(target_depth_z_m_)) {
            throw std::invalid_argument("target_depth_z_m must be finite");
        }

        rate_hz_ = std::clamp(
            declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);
        odometry_timeout_s_ = std::max(
            0.05, declare_parameter<double>("odometry_timeout_s", 0.5));

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
        target_confirmed_sub_ = create_subscription<std_msgs::msg::Bool>(
            target_confirmed_topic,
            rclcpp::QoS(1).reliable().transient_local(),
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
        vision_control_granted_pub_ = create_publisher<std_msgs::msg::Bool>(
            vision_control_granted_topic,
            rclcpp::QoS(1).reliable().transient_local());
        homing_direction_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>(
                homing_direction_topic,
                rclcpp::QoS(1).reliable().transient_local());
        emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
            emergency_stop_topic, rclcpp::QoS(1).reliable().transient_local());
        emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            emergency_stop_topic, rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&WaypointHomingControllerNode::emergency_stop_callback, this,
                std::placeholders::_1));

        publish_state();
        publish_vision_search_request(false);
        publish_vision_control_granted(false);
        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&WaypointHomingControllerNode::control_loop, this));

        if (enable_keyboard_emergency_stop && !emergency_stop_key_.empty()) {
            keyboard_thread_ = std::thread(
                &WaypointHomingControllerNode::keyboard_loop, this);
            RCLCPP_INFO(
                get_logger(),
                "Emergency hold enabled. Press '%c' to latch the current waypoint.",
                emergency_stop_key_.front());
        }

        const ArenaBounds bounds = arena_bounds(0.0);
        RCLCPP_INFO(
            get_logger(),
            "Waypoint controller ready: arena x=[%.3f, %.3f] y=[%.3f, %.3f]",
            bounds.x_min, bounds.x_max, bounds.y_min, bounds.y_max);
    }

    ~WaypointHomingControllerNode() override
    {
        stop_keyboard_thread_.store(true);
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double DEPTH_TOLERANCE_M = 0.05;

    enum class State
    {
        MOVE_TO_SCAN_CENTER,    //원형 탐색 중심 위치로 이동 상태
        MOVE_TO_SCAN_START,     //원형 궤도 시작점으로 이동 상태
        REGION_SCAN,    //원형 탐색 상태
        REGION_HOMING,    //원형 탐색 결과 그래디언트 매칭 상태
        WAIT_VISION_TARGET,    // [ACOUSTIC-VISION HANDSHAKE] 경계에서 Vision 확정을 기다린다.
        HANDOFF_COMPLETE       // [ACOUSTIC-VISION HANDSHAKE] Acoustic waypoint 생성을 종료한다.
    };

    enum class HomingWaypointResult
    {
        CREATED,
        VISION_ZONE,
        BOUNDARY
    };

    enum class RegionResultState
    {
        WAITING,
        INVALID,
        VALID
    };

    struct ArenaBounds
    {
        double x_min = 0.0;
        double x_max = 0.0;
        double y_min = 0.0;
        double y_max = 0.0;
    };

    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // 현재 위치와 yaw를 업데이트한다.
        const Eigen::Vector2d position(
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        const double z_m = msg->pose.pose.position.z;
        if (!position.allFinite() || !std::isfinite(z_m)) {
            return;
        }
        const bool first_odometry = !have_odometry_;
        current_position_ = position; // 현재 위치를 업데이트한다.
        current_z_m_ = z_m;
        odometry_frame_ = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id; // 오돔 메세지가 어느 좌표계 기준인지 업데이트한다 (빈 문자열이면 odom 기준).
        last_odometry_receive_time_ = now(); // 마지막 오도메트리 수신 시간을 업데이트한다.
        have_odometry_ = true; // 오도메트리 수신 여부를 업데이트한다.

        if (first_odometry) { // 최초 odometry 수신 시 미션을 시작한다.
            start_new_region_scan(arena_center(), true); // 최초 탐색은 실험장 중앙에서 시작.
        }
    }


    // REGION_SCAN 중에 estimator가 보낸 원형 스캔 그래디언트를 받아 저장하는 콜백
    void region_gradient_callback(
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        if (state_ != State::REGION_SCAN) {
            return;
        }
        const Eigen::Vector2d gradient(msg->vector.x, msg->vector.y);
        const double magnitude = gradient.norm(); // 그래디언트 크기
        const bool valid = gradient.allFinite() && std::isfinite(magnitude) && //유효성 검사
            magnitude > 1.0e-6;

        region_result_state_ = valid ?
            RegionResultState::VALID : RegionResultState::INVALID;
        if (valid) {
            region_gradient_ = gradient / magnitude; // 유효하면 그래디언트 방향 저장
        }
        RCLCPP_DEBUG(
            get_logger(), "REGION_SCAN gradient: valid=%s raw=(%.3f, %.3f) "
            "G_ref=(%.3f, %.3f)",
            valid ? "true" : "false", gradient.x(), gradient.y(),
            region_gradient_.x(), region_gradient_.y());
    }

    void rolling_gradient_callback(
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        if (state_ != State::REGION_HOMING || vision_search_requested_ ||
            rescan_requested_)
        {
            return;
        }
        const Eigen::Vector2d rolling(msg->vector.x, msg->vector.y);
        const double magnitude = rolling.norm();
        if (!rolling.allFinite() || magnitude <= 1.0e-6) {
            return;
        }
        const Eigen::Vector2d candidate = rolling / magnitude;
        const double angle = std::acos(std::clamp(
            homing_direction_.dot(candidate), -1.0, 1.0));
        if (angle > rolling_gradient_conflict_angle_rad_) {
            ++rolling_gradient_conflict_count_;
            RCLCPP_WARN(
                get_logger(),
                "[HOMING] rolling gradient conflict angle=%.1f deg (%zu/%zu)",
                angle * 180.0 / PI,
                rolling_gradient_conflict_count_,
                rolling_gradient_conflict_limit_);
            if (rolling_gradient_conflict_count_ >=
                rolling_gradient_conflict_limit_)
            {
                rescan_requested_ = true;
                RCLCPP_WARN(
                    get_logger(),
                    "[RESCAN] reason=rolling_gradient_conflict");
            }
            return;
        }

        rolling_gradient_conflict_count_ = 0;
        const Eigen::Vector2d blended =
            (1.0 - rolling_gradient_alpha_) * homing_direction_ +
            rolling_gradient_alpha_ * candidate;
        if (blended.norm() > 1.0e-6) {
            homing_direction_ = blended.normalized();
            publish_homing_direction();
        }
    }

    void target_confirmed_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if (!msg->data || !vision_handoff_enabled_ ||
            !vision_search_requested_ ||
            (state_ != State::REGION_HOMING && state_ != State::WAIT_VISION_TARGET))
        {
            return;
        }
        set_current_waypoint(current_position_);
        transition_to(State::HANDOFF_COMPLETE);
        publish_vision_control_granted(true);
        RCLCPP_INFO(
            get_logger(),
            "[VISION] target confirmed; hold waypoint sent and vision control granted");
    }

    bool inside_vision_zone(const Eigen::Vector2d & position) const
    {
        if (vision_near_zone_width_m_ <= 0.0) {
            return false;
        }
        const ArenaBounds bounds = arena_bounds(arena_safety_margin_m_);
        const double width = std::min(
            vision_near_zone_width_m_, bounds.y_max - bounds.y_min);
        if (arena_start_corner_ == "bottom_left") {
            return position.y() <= bounds.y_min + width;
        }
        return position.y() >= bounds.y_max - width;
    }

    bool inside_vision_zone() const
    {
        return inside_vision_zone(current_position_);
    }

    // [ACOUSTIC-VISION HANDSHAKE RESTORED] Near zone에서 Vision 확인을 요청하지만
    // target_confirmed 전까지는 Acoustic이 제어권을 유지한다.
    bool request_vision_confirmation()
    {
        if (vision_search_requested_) {
            return false;
        }
        vision_search_requested_ = true;
        publish_vision_control_granted(false);
        publish_vision_search_request(true);
        rolling_gradient_conflict_count_ = 0;
        publish_homing_direction();
        RCLCPP_INFO(
            get_logger(),
            "[VISION] target confirmation requested at position=(%.2f, %.2f) m",
            current_position_.x(), current_position_.y());
        return true;
    }

    void begin_vision_confirmation_homing()
    {
        request_vision_confirmation();
        const HomingWaypointResult result = make_next_homing_waypoint();
        if (result == HomingWaypointResult::CREATED) {
            set_current_waypoint(waypoints_.front());
            return;
        }
        begin_vision_wait();
    }

    void begin_vision_wait()
    {
        request_vision_confirmation();
        set_current_waypoint(current_position_);
        transition_to(State::WAIT_VISION_TARGET);
        RCLCPP_INFO(
            get_logger(),
            "[VISION] arena boundary reached; waiting for target confirmation");
    }

    void control_loop()
    {
        const rclcpp::Time current_time = now(); // 현재 시간을 가져온다.
        if (emergency_stop_active_.load()) {
            if (!emergency_hold_published_ && have_odometry_) {
                set_current_waypoint(current_position_);
                emergency_hold_published_ = true;
            }
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[EMERGENCY] current-position waypoint latched; restart required");
            return;
        }
        if (state_ == State::WAIT_VISION_TARGET || state_ == State::HANDOFF_COMPLETE) {
            log_controller_status();
            return;
        }
        if (!odometry_is_fresh(current_time)) {
            return;
        }
        log_controller_status();

        if (state_ == State::REGION_HOMING && vision_handoff_enabled_ &&
            inside_vision_zone() && !vision_search_requested_)
        {
            begin_vision_confirmation_homing();
            return;
        }

        switch (state_) {
            case State::MOVE_TO_SCAN_CENTER:    //원형 탐색 중심 위치로 이동하기 위한 준비 상태
                if (waypoint_reached() && depth_target_reached()) {
                    begin_move_to_scan_start();
                }
                return;

            case State::MOVE_TO_SCAN_START:
                if (waypoint_reached()) {
                    begin_region_scan();
                }
                return;

            case State::REGION_SCAN:
                if (accumulated_scan_angle_rad_ < 2.0 * PI &&
                    !update_circular_scan_waypoint())
                {
                    return;
                }
                if (region_result_state_ == RegionResultState::WAITING) {
                    return;
                }
                if (region_result_state_ == RegionResultState::INVALID) {
                    RCLCPP_WARN(
                        get_logger(), "[RESCAN] reason=invalid_region_gradient");
                    start_new_region_scan(current_position_); // 현재 위치 기준으로 새로운 원형 탐색을 요청.
                    return;
                }
                begin_region_homing(); //원형 탐색 결과 그래디언트 유효성 판정 성공이므로 원형 탐색 결과 그래디언트 매칭 상태로 전이.
                return;

            case State::REGION_HOMING:
                if (rescan_requested_) {
                    start_new_region_scan(current_position_);
                    return;
                }
                if (!waypoint_reached()) {
                    return;
                }
                handle_homing_waypoint_result(make_next_homing_waypoint());
                return;

            case State::WAIT_VISION_TARGET:
            case State::HANDOFF_COMPLETE:
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
        rescan_requested_ = false;
        rolling_gradient_conflict_count_ = 0;
        waypoints_.clear();
        transition_to(State::MOVE_TO_SCAN_CENTER, force); // 원형 탐색 중심 위치로 이동 상태로 전이.
        set_current_waypoint(scan_center_); // 원형 탐색 중심 위치를 waypoint로 설정.
    }

    void begin_move_to_scan_start()
    {
        const Eigen::Vector2d scan_start =
            scan_center_ + active_scan_radius_m_ * Eigen::Vector2d::UnitX();
        if (!waypoint_is_safe(scan_start)) {
            throw std::logic_error(
                "adjusted region scan start left arena safety bounds");
        }
        transition_to(State::MOVE_TO_SCAN_START);
        set_current_waypoint(scan_start);
    }

    void begin_region_scan()
    {
        const Eigen::Vector2d radial = current_position_ - scan_center_;
        previous_scan_angle_rad_ = std::atan2(radial.y(), radial.x());
        accumulated_scan_angle_rad_ = 0.0;
        region_result_state_ = RegionResultState::WAITING;
        transition_to(State::REGION_SCAN);
        RCLCPP_INFO(
            get_logger(),
            "[SCAN] continuous circle started center=(%.2f, %.2f) radius=%.2f m",
            scan_center_.x(), scan_center_.y(), active_scan_radius_m_);
    }

    void begin_region_homing() // 원형 탐색 결과 그래디언트 매칭 상태로 전이
    {
        rescan_requested_ = false;
        rolling_gradient_conflict_count_ = 0;
        homing_direction_ = region_gradient_;
        homing_centerline_ = current_position_;
        zigzag_sign_ = 1.0;
        transition_to(State::REGION_HOMING);
        publish_homing_direction();
        RCLCPP_INFO(
            get_logger(), "[SCAN] complete G_ref=(%.3f, %.3f)",
            region_gradient_.x(), region_gradient_.y());
        handle_homing_waypoint_result(make_next_homing_waypoint());
    }

    HomingWaypointResult make_next_homing_waypoint()
    {
        if (vision_search_requested_) {
            const Eigen::Vector2d waypoint =
                current_position_ + homing_waypoint_step_m_ * homing_direction_;
            if (!waypoint_is_safe(waypoint)) {
                return HomingWaypointResult::BOUNDARY;
            }
            waypoints_.assign(1, waypoint);
            return HomingWaypointResult::CREATED;
        }

        const Eigen::Vector2d next_centerline =
            homing_centerline_ + homing_waypoint_step_m_ * homing_direction_;
        const Eigen::Vector2d normal(
            -homing_direction_.y(), homing_direction_.x());
        const Eigen::Vector2d waypoint =
            next_centerline + zigzag_sign_ * homing_zigzag_offset_m_ * normal;

        if (vision_handoff_enabled_ && inside_vision_zone(waypoint)) {
            return HomingWaypointResult::VISION_ZONE;
        }
        if (!waypoint_is_safe(waypoint)) {
            return HomingWaypointResult::BOUNDARY;
        }

        homing_centerline_ = next_centerline;
        waypoints_.assign(1, waypoint);
        zigzag_sign_ = -zigzag_sign_;
        return HomingWaypointResult::CREATED;
    }

    void handle_homing_waypoint_result(const HomingWaypointResult result)
    {
        if (result == HomingWaypointResult::CREATED) {
            set_current_waypoint(waypoints_.front());
            return;
        }
        if (result == HomingWaypointResult::VISION_ZONE) {
            begin_vision_confirmation_homing();
            return;
        }
        handle_homing_boundary();
    }

    void handle_homing_boundary()
    {
        if (vision_search_requested_ ||
            (vision_handoff_enabled_ && inside_vision_zone()))
        {
            begin_vision_wait();
            return;
        }
        RCLCPP_WARN(get_logger(), "[RESCAN] reason=arena_boundary");
        start_new_region_scan(current_position_);
    }

    bool waypoint_reached() const
    {
        return (current_waypoint_ - current_position_).norm() <=
            waypoint_reach_tolerance_m_;
    }

    bool update_circular_scan_waypoint()
    {
        const Eigen::Vector2d radial = current_position_ - scan_center_;
        const double radius = radial.norm();
        if (radius <= 1.0e-6) {
            return false;
        }

        const double angle = std::atan2(radial.y(), radial.x());
        const double angle_delta = wrap_pi(angle - previous_scan_angle_rad_);
        if (std::abs(angle_delta) < PI / 2.0) {
            accumulated_scan_angle_rad_ += angle_delta;
        }
        previous_scan_angle_rad_ = angle;
        if (accumulated_scan_angle_rad_ >= 2.0 * PI) {
            set_current_waypoint(current_position_);
            RCLCPP_INFO(
                get_logger(), "[SCAN] continuous circle complete");
            return true;
        }

        const double target_angle = angle + scan_waypoint_lookahead_rad_;
        set_current_waypoint(
            scan_center_ + active_scan_radius_m_ *
            Eigen::Vector2d(std::cos(target_angle), std::sin(target_angle)));
        return false;
    }

    bool depth_target_reached() const
    {
        return std::abs(target_depth_z_m_ - current_z_m_) <= DEPTH_TOLERANCE_M;
    }

    void log_controller_status()
    {
        if (state_ == State::WAIT_VISION_TARGET ||
            state_ == State::HANDOFF_COMPLETE)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[CONTROL] state=%s acoustic_control=inactive",
                state_name(state_));
            return;
        }
        if (state_ == State::REGION_SCAN) {
            const double radius = (current_position_ - scan_center_).norm();
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[CONTROL] state=REGION_SCAN progress=%.0f/360 deg "
                "radius=%.2f/%.2f m z=%.2f/%.2f m",
                std::clamp(
                    accumulated_scan_angle_rad_ * 180.0 / PI, 0.0, 360.0),
                radius, active_scan_radius_m_,
                current_z_m_, target_depth_z_m_);
            return;
        }
        const double distance = (current_waypoint_ - current_position_).norm();
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[CONTROL] state=%s distance=%.2f m z=%.2f/%.2f m",
            state_name(state_), distance,
            current_z_m_, target_depth_z_m_);
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
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = arena_frame_id_;
        msg.point.x = waypoint.x() - arena_offset_x_m_;
        msg.point.y = waypoint.y() - arena_offset_y_m_;
        msg.point.z = target_depth_z_m_;
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

    // [ACOUSTIC-VISION HANDSHAKE] Vision 제어를 허용하는 최종 승인이다.
    void publish_vision_control_granted(const bool granted)
    {
        std_msgs::msg::Bool msg;
        msg.data = granted;
        vision_control_granted_pub_->publish(msg);
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

    void emergency_stop_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if (msg->data) {
            emergency_stop_active_.store(true);
        }
    }

    void trigger_emergency_stop()
    {
        if (emergency_stop_active_.exchange(true)) {
            return;
        }
        std_msgs::msg::Bool msg;
        msg.data = true;
        emergency_stop_pub_->publish(msg);
        RCLCPP_ERROR(
            get_logger(), "[EMERGENCY] key '%c' pressed; hold waypoint requested",
            emergency_stop_key_.front());
    }

    void keyboard_loop()
    {
        const int terminal_fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
        if (terminal_fd < 0) {
            RCLCPP_WARN(
                get_logger(),
                "[EMERGENCY] keyboard disabled: cannot open controlling terminal");
            return;
        }

        termios original_termios;
        bool restore_terminal = false;
        if (tcgetattr(terminal_fd, &original_termios) == 0)
        {
            termios raw_termios = original_termios;
            raw_termios.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
            raw_termios.c_cc[VMIN] = 0;
            raw_termios.c_cc[VTIME] = 0;
            restore_terminal =
                tcsetattr(terminal_fd, TCSANOW, &raw_termios) == 0;
        }

        while (rclcpp::ok() && !stop_keyboard_thread_.load()) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(terminal_fd, &read_fds);
            timeval timeout{0, 100000};
            const int ready = select(
                terminal_fd + 1, &read_fds, nullptr, nullptr, &timeout);
            if (ready <= 0 || !FD_ISSET(terminal_fd, &read_fds)) {
                continue;
            }
            char input = '\0';
            if (read(terminal_fd, &input, 1) == 1 &&
                input == emergency_stop_key_.front())
            {
                trigger_emergency_stop();
            }
        }
        if (restore_terminal) {
            tcsetattr(terminal_fd, TCSANOW, &original_termios);
        }
        close(terminal_fd);
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

    static const char * state_name(const State state)
    {
        switch (state) {
            case State::MOVE_TO_SCAN_CENTER:
                return "MOVE_TO_SCAN_CENTER";
            case State::MOVE_TO_SCAN_START:
                return "MOVE_TO_SCAN_START";
            case State::REGION_SCAN:
                return "REGION_SCAN";
            case State::REGION_HOMING:
                return "REGION_HOMING";
            case State::WAIT_VISION_TARGET:
                return "WAIT_VISION_TARGET";
            case State::HANDOFF_COMPLETE:
                return "HANDOFF_COMPLETE";
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
    double rolling_gradient_conflict_angle_rad_ = PI / 3.0;
    double waypoint_reach_tolerance_m_ = 0.15;
    double scan_waypoint_lookahead_rad_ = 0.35;
    double vision_near_zone_width_m_ = 2.0;
    double target_depth_z_m_ = -0.65;
    double rate_hz_ = 30.0;
    double odometry_timeout_s_ = 0.5;
    std::size_t rolling_gradient_conflict_limit_ = 3;
    std::size_t rolling_gradient_conflict_count_ = 0;
    double zigzag_sign_ = 1.0;
    std::string arena_start_corner_ = "bottom_left";
    std::string arena_frame_id_ = "arena";
    std::string odometry_frame_ = "odom";
    bool vision_handoff_enabled_ = true;
    bool have_odometry_ = false;
    bool rescan_requested_ = false;
    bool vision_search_requested_ = false;
    bool first_region_scan_ = true;
    bool emergency_hold_published_ = false;
    std::string emergency_stop_key_ = "s";
    State state_ = State::MOVE_TO_SCAN_CENTER;
    RegionResultState region_result_state_ = RegionResultState::WAITING;
    rclcpp::Time last_odometry_receive_time_;
    double current_z_m_ = 0.0;
    double previous_scan_angle_rad_ = 0.0;
    double accumulated_scan_angle_rad_ = 0.0;
    Eigen::Vector2d current_position_{0.0, 0.0};
    Eigen::Vector2d scan_center_{0.0, 0.0};
    Eigen::Vector2d current_waypoint_{0.0, 0.0};
    Eigen::Vector2d homing_centerline_{0.0, 0.0};
    Eigen::Vector2d region_gradient_{1.0, 0.0};
    Eigen::Vector2d homing_direction_{1.0, 0.0};
    std::vector<Eigen::Vector2d> waypoints_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        region_gradient_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        rolling_gradient_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_confirmed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr scan_center_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_search_request_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_control_granted_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        homing_direction_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic_bool emergency_stop_active_{false};
    std::atomic_bool stop_keyboard_thread_{false};
    std::thread keyboard_thread_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::WaypointHomingControllerNode)
