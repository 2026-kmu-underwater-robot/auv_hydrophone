#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace audio_capture
{
// MuJoCo 절대 odometry를 첫 수신 XY가 (0,0)이 되도록 리베이스한다.
// z와 orientation은 raw 값을 유지한다. start_frame은 다루지 않는다.
class SimOdometryRebaserNode : public rclcpp::Node
{
public:
    explicit SimOdometryRebaserNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("sim_odometry_rebaser", options)
    {
        const auto input_topic = declare_parameter<std::string>(
            "input_topic", "/odometry/mujoco_raw");
        const auto output_topic = declare_parameter<std::string>(
            "output_topic", "/odometry/filtered");
        output_frame_ = declare_parameter<std::string>("output_frame", "odom");

        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            input_topic, 30,
            std::bind(
                &SimOdometryRebaserNode::odometry_callback, this,
                std::placeholders::_1));
        odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 30);

        RCLCPP_INFO(
            get_logger(),
            "sim_odometry_rebaser: %s -> %s (XY zeroed at first sample, z/yaw kept)",
            input_topic.c_str(), output_topic.c_str());
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        const Eigen::Vector2d position(
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        const double z = msg->pose.pose.position.z;
        if (!position.allFinite() || !std::isfinite(z)) {
            return;
        }

        if (!have_origin_) {
            origin_ = position;
            have_origin_ = true;
            RCLCPP_INFO(
                get_logger(),
                "Captured sim odom origin_xy=(%.3f, %.3f); subsequent poses "
                "are published relative to this XY with raw z",
                origin_.x(), origin_.y());
        }

        const Eigen::Vector2d relative = position - origin_;
        nav_msgs::msg::Odometry output = *msg;
        output.header.frame_id = output_frame_;
        output.pose.pose.position.x = relative.x();
        output.pose.pose.position.y = relative.y();
        output.pose.pose.position.z = z;
        odometry_pub_->publish(output);
    }

    bool have_origin_ = false;
    std::string output_frame_ = "odom";
    Eigen::Vector2d origin_{0.0, 0.0};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::SimOdometryRebaserNode)
