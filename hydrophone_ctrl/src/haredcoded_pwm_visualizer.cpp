#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hydrophone_ctrl
{

class HaredcodedPwmVisualizerNode : public rclcpp::Node
{
public:
    HaredcodedPwmVisualizerNode()
    : Node("haredcoded_pwm_visualizer")
    {
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        time_scale_ = declare_parameter<double>("time_scale_m_per_sec", 0.4);
        snr_scale_ = declare_parameter<double>("snr_scale_m_per_db", 0.05);
        origin_x_ = declare_parameter<double>("origin_x", 0.0);
        origin_y_ = declare_parameter<double>("origin_y", 0.0);
        origin_z_ = declare_parameter<double>("origin_z", 0.2);

        marker_publisher_ =
            create_publisher<visualization_msgs::msg::MarkerArray>(
                "/homing/haredcoded_pwm_markers", 10);

        state_subscription_ = create_subscription<std_msgs::msg::String>(
            "/homing/haredcoded_pwm_state",
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::String::ConstSharedPtr msg)
            {
                if (msg->data == "SNR_FORWARD" && state_ != "SNR_FORWARD") {
                    raw_samples_.clear();
                    filtered_samples_.clear();
                    have_target_ = false;
                }
                state_ = msg->data;
            });
        raw_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/homing/snr_raw_time_map", 20,
            [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
            {
                raw_samples_.push_back(msg->point);
            });
        filtered_subscription_ =
            create_subscription<geometry_msgs::msg::PointStamped>(
                "/homing/snr_filtered_time_map", 20,
                [this](
                    const geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
                {
                    filtered_samples_.push_back(msg->point);
                });
        target_subscription_ =
            create_subscription<geometry_msgs::msg::PointStamped>(
                "/homing/snr_selected_time_target",
                rclcpp::QoS(1).reliable().transient_local(),
                [this](
                    const geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
                {
                    selected_target_ = msg->point;
                    have_target_ = true;
                });

        vision_request_subscription_ = bool_subscription(
            "/homing/vision_search_active",
            &HaredcodedPwmVisualizerNode::vision_requested_);
        vision_granted_subscription_ = bool_subscription(
            "/homing/vision_control_granted",
            &HaredcodedPwmVisualizerNode::vision_granted_);
        timeout_subscription_ = bool_subscription(
            "/homing/timeout", &HaredcodedPwmVisualizerNode::timed_out_);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), [this]() {publish_markers();});
    }

private:
    using Marker = visualization_msgs::msg::Marker;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscription(
        const std::string & topic,
        bool HaredcodedPwmVisualizerNode::* destination)
    {
        return create_subscription<std_msgs::msg::Bool>(
            topic, rclcpp::QoS(1).reliable().transient_local(),
            [this, destination](const std_msgs::msg::Bool::ConstSharedPtr msg)
            {
                this->*destination = msg->data;
            });
    }

    Marker marker(const int id, const int type, const std::string & name_space)
    {
        Marker result;
        result.header.stamp = now();
        result.header.frame_id = frame_id_;
        result.ns = name_space;
        result.id = id;
        result.type = type;
        result.action = Marker::ADD;
        result.pose.orientation.w = 1.0;
        return result;
    }

    geometry_msgs::msg::Point plot_point(
        const geometry_msgs::msg::Point & sample, const double minimum_snr,
        const double z_offset) const
    {
        geometry_msgs::msg::Point point;
        point.x = origin_x_ + sample.x * time_scale_;
        point.y = origin_y_ + (sample.y - minimum_snr) * snr_scale_;
        point.z = origin_z_ + z_offset;
        return point;
    }

    static std_msgs::msg::ColorRGBA heatmap_color(
        const double snr, const double minimum_snr, const double maximum_snr)
    {
        const double normalized = std::clamp(
            (snr - minimum_snr) / (maximum_snr - minimum_snr), 0.0, 1.0);
        const auto jet = [normalized](const double center) {
            return static_cast<float>(std::clamp(
                1.5 - std::abs(4.0 * normalized - center), 0.0, 1.0));
        };
        std_msgs::msg::ColorRGBA color;
        color.r = jet(3.0);
        color.g = jet(2.0);
        color.b = jet(1.0);
        color.a = 1.0F;
        return color;
    }

    void publish_markers()
    {
        double minimum_snr = std::numeric_limits<double>::infinity();
        double maximum_snr = -std::numeric_limits<double>::infinity();
        double maximum_time = 1.0;
        const auto update_bounds = [&](const geometry_msgs::msg::Point & sample) {
            minimum_snr = std::min(minimum_snr, sample.y);
            maximum_snr = std::max(maximum_snr, sample.y);
            maximum_time = std::max(maximum_time, sample.x);
        };
        for (const auto & sample : raw_samples_) {
            update_bounds(sample);
        }
        for (const auto & sample : filtered_samples_) {
            update_bounds(sample);
        }
        if (have_target_) {
            update_bounds(selected_target_);
        }
        if (!std::isfinite(minimum_snr)) {
            minimum_snr = 0.0;
            maximum_snr = 10.0;
        }
        if (maximum_snr - minimum_snr < 1.0) {
            maximum_snr = minimum_snr + 1.0;
        }

        visualization_msgs::msg::MarkerArray array;

        auto axes = marker(0, Marker::LINE_LIST, "snr_axes");
        axes.scale.x = 0.025;
        axes.color.r = 0.7F;
        axes.color.g = 0.7F;
        axes.color.b = 0.7F;
        axes.color.a = 1.0F;
        geometry_msgs::msg::Point origin;
        origin.x = origin_x_;
        origin.y = origin_y_;
        origin.z = origin_z_;
        auto time_end = origin;
        time_end.x += maximum_time * time_scale_;
        auto snr_end = origin;
        snr_end.y += (maximum_snr - minimum_snr) * snr_scale_;
        axes.points = {origin, time_end, origin, snr_end};
        array.markers.push_back(axes);

        auto raw = marker(1, Marker::POINTS, "snr_raw");
        raw.scale.x = 0.055;
        raw.scale.y = 0.055;
        raw.color.r = 1.0F;
        raw.color.g = 0.55F;
        raw.color.b = 0.1F;
        raw.color.a = 0.9F;
        for (const auto & sample : raw_samples_) {
            raw.points.push_back(plot_point(sample, minimum_snr, 0.02));
        }
        array.markers.push_back(raw);

        auto filtered = marker(2, Marker::LINE_STRIP, "snr_median");
        filtered.scale.x = 0.045;
        filtered.color.r = 0.1F;
        filtered.color.g = 1.0F;
        filtered.color.b = 0.25F;
        filtered.color.a = 1.0F;
        for (const auto & sample : filtered_samples_) {
            filtered.points.push_back(plot_point(sample, minimum_snr, 0.04));
        }
        array.markers.push_back(filtered);

        auto target = marker(3, Marker::LINE_LIST, "snr_target");
        target.scale.x = 0.06;
        target.color.r = 1.0F;
        target.color.b = 1.0F;
        target.color.a = have_target_ ? 1.0F : 0.0F;
        if (have_target_) {
            auto bottom = plot_point(selected_target_, minimum_snr, 0.06);
            bottom.y = origin_y_;
            auto top = bottom;
            top.y = origin_y_ + (maximum_snr - minimum_snr) * snr_scale_;
            target.points = {bottom, top};
        }
        array.markers.push_back(target);

        auto heatmap = marker(4, Marker::CUBE_LIST, "snr_heatmap");
        double sample_period_sec = 0.1;
        if (raw_samples_.size() >= 2) {
            sample_period_sec = std::max(
                0.01, raw_samples_.back().x - raw_samples_[raw_samples_.size() - 2].x);
        }
        heatmap.scale.x = std::max(0.02, sample_period_sec * time_scale_ * 0.9);
        heatmap.scale.y = 0.22;
        heatmap.scale.z = 0.06;
        for (const auto & sample : raw_samples_) {
            geometry_msgs::msg::Point point;
            point.x = origin_x_ + sample.x * time_scale_;
            point.y = origin_y_ - 0.22;
            point.z = origin_z_ + 0.03;
            heatmap.points.push_back(point);
            heatmap.colors.push_back(
                heatmap_color(sample.y, minimum_snr, maximum_snr));
        }
        array.markers.push_back(heatmap);

        auto status = marker(5, Marker::TEXT_VIEW_FACING, "status");
        status.pose.position.x = origin_x_;
        status.pose.position.y = origin_y_ - 0.65;
        status.pose.position.z = origin_z_ + 0.25;
        status.scale.z = 0.22;
        status.color.r = timed_out_ ? 1.0F : 0.9F;
        status.color.g = timed_out_ ? 0.1F : 0.9F;
        status.color.b = timed_out_ ? 0.1F : 0.9F;
        status.color.a = 1.0F;
        std::ostringstream text;
        text << "state: " << state_
             << " | vision request: " << (vision_requested_ ? "ON" : "OFF")
             << " | granted: " << (vision_granted_ ? "YES" : "NO")
             << " | timeout: " << (timed_out_ ? "YES" : "NO");
        if (have_target_) {
            text << std::fixed << std::setprecision(2)
                 << " | target: " << selected_target_.x << " s, "
                 << selected_target_.y << " dB";
        }
        status.text = text.str();
        array.markers.push_back(status);

        marker_publisher_->publish(array);
    }

    std::string frame_id_ = "map";
    double time_scale_ = 0.4;
    double snr_scale_ = 0.05;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double origin_z_ = 0.2;
    std::string state_ = "WAITING";
    bool vision_requested_ = false;
    bool vision_granted_ = false;
    bool timed_out_ = false;
    bool have_target_ = false;
    geometry_msgs::msg::Point selected_target_;
    std::vector<geometry_msgs::msg::Point> raw_samples_;
    std::vector<geometry_msgs::msg::Point> filtered_samples_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        raw_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        filtered_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        target_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        vision_request_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        vision_granted_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr timeout_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hydrophone_ctrl

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<hydrophone_ctrl::HaredcodedPwmVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
