#include <cmath>
#include <functional>
#include <string>

#include <Eigen/Dense>
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
        output_frame_ = declare_parameter<std::string>("output_frame", "odom");
        arena_yaw_rad_ = declare_parameter<double>(
            "arena_yaw_rad", 3.14159265358979323846);

        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            input_topic, 30,
            std::bind(&SimOdometryRebaserNode::odometry_callback, this,
                std::placeholders::_1));
        odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 30);
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
            have_origin_ = true;
        }

        const Eigen::Vector2d delta = position - origin_;
        const double c = std::cos(arena_yaw_rad_);
        const double s = std::sin(arena_yaw_rad_);
        const Eigen::Vector2d arena_position(
            c * delta.x() + s * delta.y(),
            -s * delta.x() + c * delta.y());

        nav_msgs::msg::Odometry output = *msg;
        output.header.frame_id = output_frame_;
        output.pose.pose.position.x = arena_position.x();
        output.pose.pose.position.y = arena_position.y();

        // world frame을 -arena_yaw만큼 회전하므로 orientation에도 같은 회전을 적용한다.
        const double half = -0.5 * arena_yaw_rad_;
        const double ow = std::cos(half);
        const double oz = std::sin(half);
        const auto & q = msg->pose.pose.orientation;
        output.pose.pose.orientation.w = ow * q.w - oz * q.z;
        output.pose.pose.orientation.x = ow * q.x - oz * q.y;
        output.pose.pose.orientation.y = ow * q.y + oz * q.x;
        output.pose.pose.orientation.z = ow * q.z + oz * q.w;
        odometry_pub_->publish(output);
    }

    double arena_yaw_rad_ = 3.14159265358979323846;
    bool have_origin_ = false;
    std::string output_frame_ = "odom";
    Eigen::Vector2d origin_{0.0, 0.0};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::SimOdometryRebaserNode)
