#include <cmath>
#include <functional>
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
        arena_yaw_rad_ = declare_parameter<double>(
            "arena_yaw_rad", 3.14159265358979323846);

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
            have_origin_ = true;
            if (pending_start_frame_) {
                publish_rebased_start_frame(*pending_start_frame_);
                pending_start_frame_.reset();
            }
        }

        const Eigen::Vector2d arena_position = rebase_position(position);

        nav_msgs::msg::Odometry output = *msg;
        output.header.frame_id = output_frame_;
        output.pose.pose.position.x = arena_position.x();
        output.pose.pose.position.y = arena_position.y();

        output.pose.pose.orientation =
            rebase_orientation(msg->pose.pose.orientation);
        odometry_pub_->publish(output);
    }

    void start_frame_callback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        if (!have_origin_) {
            pending_start_frame_ = *msg;
            return;
        }
        publish_rebased_start_frame(*msg);
    }

    void publish_rebased_start_frame(
        const geometry_msgs::msg::PoseStamped & input)
    {
        geometry_msgs::msg::PoseStamped output = input;
        output.header.frame_id = output_frame_;
        const Eigen::Vector2d rebased = rebase_position(
            {input.pose.position.x, input.pose.position.y});
        output.pose.position.x = rebased.x();
        output.pose.position.y = rebased.y();
        output.pose.orientation = rebase_orientation(input.pose.orientation);
        start_frame_pub_->publish(output);
    }

    Eigen::Vector2d rebase_position(const Eigen::Vector2d & position) const
    {
        const Eigen::Vector2d delta = position - origin_;
        const double cosine = std::cos(arena_yaw_rad_);
        const double sine = std::sin(arena_yaw_rad_);
        return {
            cosine * delta.x() + sine * delta.y(),
            -sine * delta.x() + cosine * delta.y()};
    }

    geometry_msgs::msg::Quaternion rebase_orientation(
        const geometry_msgs::msg::Quaternion & input) const
    {
        // world frame을 -arena_yaw만큼 회전하므로 orientation에도 같은 회전을 적용한다.
        const double half = -0.5 * arena_yaw_rad_;
        const double rotation_w = std::cos(half);
        const double rotation_z = std::sin(half);
        geometry_msgs::msg::Quaternion output;
        output.w = rotation_w * input.w - rotation_z * input.z;
        output.x = rotation_w * input.x - rotation_z * input.y;
        output.y = rotation_w * input.y + rotation_z * input.x;
        output.z = rotation_w * input.z + rotation_z * input.w;
        return output;
    }

    double arena_yaw_rad_ = 3.14159265358979323846;
    bool have_origin_ = false;
    std::string output_frame_ = "odom";
    Eigen::Vector2d origin_{0.0, 0.0};
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
