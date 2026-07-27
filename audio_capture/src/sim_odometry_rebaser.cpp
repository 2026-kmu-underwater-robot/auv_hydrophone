#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace audio_capture
{
// MuJoCo 절대 odometry를 첫 수신 위치가 (0,0)이 되는 homing 전용 arena
// odometry로 변환한다. 실제 운용 odometry 경로에는 사용하지 않는다.
class SimOdometryRebaserNode : public rclcpp::Node
{
public:
    explicit SimOdometryRebaserNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("sim_odometry_rebaser", options)
    {
        const auto input_topic = declare_parameter<std::string>(
            "input_topic", "/odometry/filtered");
        const auto output_topic = declare_parameter<std::string>(
            "output_topic", "/homing/sim_odometry");
        const auto start_frame_input_topic = declare_parameter<std::string>(
            "start_frame_input_topic", "/guided/start_frame");
        const auto start_frame_output_topic = declare_parameter<std::string>(
            "start_frame_output_topic", "/homing/sim_start_frame");
        output_frame_ = declare_parameter<std::string>("output_frame", "odom");
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            input_topic, 30,
            std::bind(&SimOdometryRebaserNode::odometry_callback, this,
                std::placeholders::_1));
        odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 30);
        start_frame_sub_ =
            create_subscription<geometry_msgs::msg::PoseStamped>(
                start_frame_input_topic,
                rclcpp::QoS(1).reliable().transient_local(),
                std::bind(
                    &SimOdometryRebaserNode::start_frame_callback, this,
                    std::placeholders::_1));
        start_frame_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            start_frame_output_topic,
            rclcpp::QoS(1).reliable().transient_local());
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        const Eigen::Vector2d position(
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        if (!position.allFinite()) {
            return;
        }
        if (!have_origin_) {
            origin_ = position;
            initial_raw_yaw_rad_ = yaw_from_quaternion(
                msg->pose.pose.orientation);
            if (!std::isfinite(initial_raw_yaw_rad_)) {
                return;
            }
            have_origin_ = true;
            if (pending_start_frame_) {
                configure_start_frame(*pending_start_frame_);
                pending_start_frame_.reset();
            }
        }
        if (!start_frame_ready_) {
            return;
        }

        const Eigen::Vector2d arena_position = rebase_position(position);

        nav_msgs::msg::Odometry output = *msg;
        output.header.frame_id = output_frame_;
        output.pose.pose.position.x = arena_position.x();
        output.pose.pose.position.y = arena_position.y();
        output.pose.pose.orientation =
            rotate_orientation(msg->pose.pose.orientation);
        odometry_pub_->publish(output);
    }

    void start_frame_callback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        if (!have_origin_) {
            pending_start_frame_ = *msg;
            return;
        }
        configure_start_frame(*msg);
    }

    void configure_start_frame(
        const geometry_msgs::msg::PoseStamped & input)
    {
        const double absolute_start_yaw_rad =
            yaw_from_quaternion(input.pose.orientation);
        if (!std::isfinite(absolute_start_yaw_rad)) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid guided start frame yaw");
            return;
        }
        yaw_offset_rad_ =
            absolute_start_yaw_rad - initial_raw_yaw_rad_;
        start_frame_ready_ = true;

        geometry_msgs::msg::PoseStamped output = input;
        output.header.frame_id = output_frame_;
        start_frame_pub_->publish(output);
        RCLCPP_INFO(
            get_logger(),
            "Simulation odom aligned: raw initial yaw=%.3f rad, "
            "absolute odom yaw=%.3f rad, yaw offset=%.3f rad",
            initial_raw_yaw_rad_, absolute_start_yaw_rad, yaw_offset_rad_);
    }

    Eigen::Vector2d rebase_position(const Eigen::Vector2d & position) const
    {
        const Eigen::Vector2d delta = position - origin_;
        const double cosine = std::cos(yaw_offset_rad_);
        const double sine = std::sin(yaw_offset_rad_);
        return {
            cosine * delta.x() - sine * delta.y(),
            sine * delta.x() + cosine * delta.y()};
    }

    geometry_msgs::msg::Quaternion rotate_orientation(
        const geometry_msgs::msg::Quaternion & input) const
    {
        const double half = 0.5 * yaw_offset_rad_;
        const double rotation_w = std::cos(half);
        const double rotation_z = std::sin(half);
        geometry_msgs::msg::Quaternion output;
        output.w = rotation_w * input.w - rotation_z * input.z;
        output.x = rotation_w * input.x - rotation_z * input.y;
        output.y = rotation_w * input.y + rotation_z * input.x;
        output.z = rotation_w * input.z + rotation_z * input.w;
        return output;
    }

    static double yaw_from_quaternion(
        const geometry_msgs::msg::Quaternion & orientation)
    {
        const double norm = std::sqrt(
            orientation.x * orientation.x +
            orientation.y * orientation.y +
            orientation.z * orientation.z +
            orientation.w * orientation.w);
        if (!std::isfinite(norm) || norm <= 1.0e-9) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double x = orientation.x / norm;
        const double y = orientation.y / norm;
        const double z = orientation.z / norm;
        const double w = orientation.w / norm;
        return std::atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z));
    }

    bool have_origin_ = false;
    bool start_frame_ready_ = false;
    std::string output_frame_ = "odom";
    Eigen::Vector2d origin_{0.0, 0.0};
    double initial_raw_yaw_rad_ = 0.0;
    double yaw_offset_rad_ = 0.0;
    std::optional<geometry_msgs::msg::PoseStamped> pending_start_frame_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        start_frame_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        start_frame_pub_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::SimOdometryRebaserNode)
