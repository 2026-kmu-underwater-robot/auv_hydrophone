#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <audio_common_msgs/msg/float64_stamped.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

namespace audio_capture
{
// 전체 알고리즘 흐름:
//   1. 각 SNR 측정값을 가장 최근의 기체 위치와 대응시킨다.
//   2. 공간적으로 떨어진 표본만 제한된 시간/개수 구간에 보관한다.
//   3. 최소제곱법으로 국소 SNR gradient를 추정한다. d(SNR)/dr < 0이면
//      SNR이 증가하는 gradient 방향이 음원을 향한다.
//   4. 기체 이동, 후보까지의 거리 변화, SNR 변화의 부호 관계를 이용해
//      선택적으로 음원 후보 particle의 가중치를 갱신한다.
//   5. gradient, particle 또는 혼합 방향을 선택하고 평활화한 뒤
//      odometry 좌표계 또는 기체 좌표계로 발행한다.
class SnrGradientHomingNode : public rclcpp::Node
{
public:
    explicit SnrGradientHomingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("snr_gradient_homing", options),
      rng_(static_cast<std::uint32_t>(declare_parameter<std::int64_t>("random_seed", 7)))
    {
        // 설정 분기 A: ROS 입력/출력 토픽.
        snr_topic_ = declare_parameter<std::string>(
            "snr_topic", "/audio_phase_estimator/iq_snr_ratio_stamped");
        odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
        depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth/pose");
        direction_topic_ = declare_parameter<std::string>("direction_topic", "/homing/direction");
        confidence_topic_ = declare_parameter<std::string>("confidence_topic", "/homing/snr_confidence");
        source_estimate_topic_ =
            declare_parameter<std::string>("source_estimate_topic", "/homing/source_estimate");
        estimator_ready_topic_ =
            declare_parameter<std::string>("estimator_ready_topic", "/homing/estimator_ready");
        near_source_topic_ =
            declare_parameter<std::string>("near_source_topic", "/homing/near_source");
        reset_topic_ = declare_parameter<std::string>("reset_topic", "/homing/reset_estimator");

        // 설정 분기 B: 위치 정보 출처와 방향 출력 좌표계.
        output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
        output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");
        use_depth_pose_ = declare_parameter<bool>("use_depth_pose", true);
        horizontal_only_ = declare_parameter<bool>("horizontal_only", true);

        // 설정 분기 C: 공간 표본 구간과 gradient 추정기.
        max_sample_count_ = static_cast<std::size_t>(
            std::max<std::int64_t>(3, declare_parameter<std::int64_t>("max_sample_count", 160)));
        min_sample_count_ = static_cast<std::size_t>(
            std::max<std::int64_t>(3, declare_parameter<std::int64_t>("min_sample_count", 16)));
        max_sample_age_s_ = std::max(0.1, declare_parameter<double>("max_sample_age_s", 45.0));
        min_sample_spacing_m_ = std::max(0.0, declare_parameter<double>("min_sample_spacing_m", 0.03));
        min_motion_baseline_m_ = std::max(0.0, declare_parameter<double>("min_motion_baseline_m", 0.30));
        min_snr_span_ = std::max(0.0, declare_parameter<double>("min_snr_span", 0.20));
        snr_deadband_ = std::max(0.0, declare_parameter<double>("snr_deadband", 0.02));
        direction_filter_alpha_ =
            std::clamp(declare_parameter<double>("direction_filter_alpha", 0.25), 0.0, 1.0);
        covariance_regularization_ =
            std::max(1.0e-12, declare_parameter<double>("covariance_regularization", 1.0e-4));
        min_trajectory_coverage_ratio_ = std::clamp(
            declare_parameter<double>("min_trajectory_coverage_ratio", 0.12), 0.0, 1.0);
        gradient_stability_window_ = static_cast<std::size_t>(std::max<std::int64_t>(
            3, declare_parameter<std::int64_t>("gradient_stability_window", 8)));
        max_gradient_std_rad_ = std::clamp(
            declare_parameter<double>("max_gradient_std_rad", 0.30), 0.01, PI);
        snr_median_window_ = static_cast<std::size_t>(std::max<std::int64_t>(
            1, declare_parameter<std::int64_t>("snr_median_window", 5)));
        odometry_history_age_s_ = std::max(
            1.0, declare_parameter<double>("odometry_history_age_s", 10.0));
        max_odometry_extrapolation_s_ = std::max(
            0.0, declare_parameter<double>("max_odometry_extrapolation_s", 0.10));
        near_source_distance_m_ = std::max(
            0.0, declare_parameter<double>("near_source_distance_m", 1.0));
        near_source_min_snr_ = declare_parameter<double>("near_source_min_snr", 5.0);
        near_source_max_snr_delta_ = std::max(
            0.0, declare_parameter<double>("near_source_max_snr_delta", 0.10));

        // 설정 분기 D: 선택적인 particle filter 일관성 검사.
        enable_particle_filter_ = declare_parameter<bool>("enable_particle_filter", true);
        direction_source_ = declare_parameter<std::string>("direction_source", "blend");
        particle_count_ = static_cast<std::size_t>(
            std::max<std::int64_t>(10, declare_parameter<std::int64_t>("particle_count", 500)));
        particle_radius_min_m_ = std::max(0.0, declare_parameter<double>("particle_radius_min_m", 0.5));
        particle_radius_max_m_ =
            std::max(particle_radius_min_m_ + 0.1, declare_parameter<double>("particle_radius_max_m", 15.0));
        particle_update_gain_ =
            std::max(0.0, declare_parameter<double>("particle_update_gain", 0.8));
        particle_resample_ess_ratio_ =
            std::clamp(declare_parameter<double>("particle_resample_ess_ratio", 0.45), 0.05, 1.0);
        particle_blend_ = std::clamp(declare_parameter<double>("particle_blend", 0.35), 0.0, 1.0);
        particle_motion_deadband_m_ =
            std::max(0.0, declare_parameter<double>("particle_motion_deadband_m", 0.02));
        particle_snr_scale_ =
            std::max(1.0e-6, declare_parameter<double>("particle_snr_scale", 0.5));
        particle_roughening_std_m_ =
            std::max(0.0, declare_parameter<double>("particle_roughening_std_m", 0.20));
        particle_cluster_radius_m_ =
            std::max(0.1, declare_parameter<double>("particle_cluster_radius_m", 2.0));
        particle_prior_strength_ =
            std::max(0.0, declare_parameter<double>("particle_prior_strength", 2.0));
        require_particle_agreement_ = declare_parameter<bool>("require_particle_agreement", false);
        particle_agreement_min_dot_ =
            std::clamp(declare_parameter<double>("particle_agreement_min_dot", 0.0), -1.0, 1.0);

        // 실행 입력: SNR이 추정을 구동하고 위치 callback이 현재 상태를 유지한다.
        snr_sub_ = create_subscription<audio_common_msgs::msg::Float64Stamped>(
            snr_topic_,
            10,
            std::bind(&SnrGradientHomingNode::snr_callback, this, std::placeholders::_1));
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_,
            20,
            std::bind(&SnrGradientHomingNode::odometry_callback, this, std::placeholders::_1));
        // 분기: 별도 수심 위치를 사용하거나 odometry에서 z를 직접 가져온다.
        if (use_depth_pose_) {
            depth_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                depth_topic_,
                20,
                std::bind(&SnrGradientHomingNode::depth_callback, this, std::placeholders::_1));
        }

        direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(direction_topic_, 10);
        confidence_pub_ = create_publisher<std_msgs::msg::Float64>(confidence_topic_, 10);
        source_estimate_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>(source_estimate_topic_, 10);
        estimator_ready_pub_ = create_publisher<std_msgs::msg::Bool>(estimator_ready_topic_, 10);
        near_source_pub_ = create_publisher<std_msgs::msg::Bool>(near_source_topic_, 10);
        reset_sub_ = create_subscription<std_msgs::msg::Empty>(
            reset_topic_, 10, std::bind(&SnrGradientHomingNode::reset_callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "SNR gradient homing ready. snr=%s odom=%s direction=%s source=%s particles=%s",
            snr_topic_.c_str(),
            odometry_topic_.c_str(),
            direction_topic_.c_str(),
            direction_source_.c_str(),
            enable_particle_filter_ ? "on" : "off");
    }

private:
    static constexpr double PI = 3.14159265358979323846;

    struct Sample
    {
        rclcpp::Time stamp;
        Eigen::Vector3d position_m{0.0, 0.0, 0.0};
        double snr = 0.0;
    };

    struct Particle
    {
        Eigen::Vector3d position_m{0.0, 0.0, 0.0};
        double weight = 1.0;
    };

    struct PoseSample
    {
        rclcpp::Time stamp;
        Eigen::Vector3d position_m{0.0, 0.0, 0.0};
    };

    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // 1단계: 가장 최근의 수평 위치와 기체 방위를 저장한다.
        current_position_m_.x() = msg->pose.pose.position.x;
        current_position_m_.y() = msg->pose.pose.position.y;
        // 분기: 별도 수심 토픽을 사용하지 않을 때만 odometry가 수심을 제공한다.
        if (!use_depth_pose_) {
            current_position_m_.z() = msg->pose.pose.position.z;
        }
        current_yaw_rad_ = yaw_from_quaternion(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        odometry_frame_id_ = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;
        have_pose_ = true;
        const rclcpp::Time stamp(msg->header.stamp);
        if (stamp.nanoseconds() > 0) {
            // rosbag을 다시 처음부터 재생하면 timestamp가 뒤로 이동한다.
            // 미래 시각의 odometry/gradient/PF 상태를 새 재생에 섞지 않는다.
            if (!odometry_history_.empty() && stamp < odometry_history_.back().stamp) {
                odometry_history_.clear();
                reset_estimator_state();
                RCLCPP_INFO(get_logger(), "Odometry time moved backwards; estimator state reset.");
            }
            odometry_history_.push_back({stamp, current_position_m_});
            while (!odometry_history_.empty() &&
                (stamp - odometry_history_.front().stamp).seconds() > odometry_history_age_s_)
            {
                odometry_history_.pop_front();
            }
        }
    }

    void depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
    {
        // 1-b단계: 별도 수심 추정값으로 z를 갱신한다.
        current_position_m_.z() = msg->pose.pose.position.z;
        have_depth_ = true;
    }

    void snr_callback(const audio_common_msgs::msg::Float64Stamped::ConstSharedPtr msg)
    {
        // 2-a단계: 설정된 위치 입력이 모두 준비될 때까지 SNR을 폐기한다.
        if (!have_pose_ || (use_depth_pose_ && !have_depth_)) {
            return;
        }
        // 분기: NaN/Inf는 회귀 계산과 particle 가중치 계산에서 제외한다.
        if (!std::isfinite(msg->data)) {
            return;
        }

        const rclcpp::Time measurement_stamp(msg->header.stamp);
        Eigen::Vector3d measurement_position;
        if (measurement_stamp.nanoseconds() <= 0 ||
            !interpolate_odometry(measurement_stamp, measurement_position))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "No odometry sample for stamped SNR measurement; sample dropped.");
            return;
        }

        snr_median_buffer_.push_back(msg->data);
        while (snr_median_buffer_.size() > snr_median_window_) {
            snr_median_buffer_.pop_front();
        }
        std::vector<double> sorted_snr(snr_median_buffer_.begin(), snr_median_buffer_.end());
        std::sort(sorted_snr.begin(), sorted_snr.end());
        const double filtered_snr = sorted_snr[sorted_snr.size() / 2];

        Sample sample;
        sample.stamp = measurement_stamp;
        sample.position_m = measurement_position;
        sample.snr = filtered_snr;
        // 분기: 거의 같은 위치에서는 공간 기준점을 유지하고 측정값만 갱신한다.
        if (!samples_.empty() &&
            (sample.position_m - samples_.back().position_m).norm() < min_sample_spacing_m_)
        {
            // 공간 기준점은 유지해야 누적 이동 거리가 spacing을 넘을 수 있다.
            // 위치까지 현재 값으로 교체하면 고주기 SNR 입력에서 기준점이 기체를
            // 계속 따라가므로 새 공간 표본이 영원히 추가되지 않는다.
            samples_.back().stamp = sample.stamp;
            samples_.back().snr = sample.snr;
            return;
        }

        // 2-b단계: 현재 표본을 추가하기 전에 음원 후보 위치를 갱신한다.
        if (enable_particle_filter_) {
            update_particles(sample);
        }
        // 2-c단계: 관측 구간을 정리한 뒤 새로운 방향을 추정한다.
        samples_.push_back(sample);
        prune_samples(sample.stamp);
        update_direction(sample.stamp);
    }

    bool interpolate_odometry(const rclcpp::Time & stamp, Eigen::Vector3d & position) const
    {
        if (odometry_history_.empty()) {
            return false;
        }
        if (stamp <= odometry_history_.front().stamp) {
            if ((odometry_history_.front().stamp - stamp).seconds() > max_odometry_extrapolation_s_) {
                return false;
            }
            position = odometry_history_.front().position_m;
            return true;
        }
        if (stamp >= odometry_history_.back().stamp) {
            if ((stamp - odometry_history_.back().stamp).seconds() > max_odometry_extrapolation_s_) {
                return false;
            }
            position = odometry_history_.back().position_m;
            return true;
        }
        for (std::size_t i = 1; i < odometry_history_.size(); ++i) {
            if (odometry_history_[i].stamp >= stamp) {
                const PoseSample & before = odometry_history_[i - 1];
                const PoseSample & after = odometry_history_[i];
                const double span = (after.stamp - before.stamp).seconds();
                const double alpha = span > 0.0 ? (stamp - before.stamp).seconds() / span : 0.0;
                position = (1.0 - alpha) * before.position_m + alpha * after.position_m;
                return true;
            }
        }
        return false;
    }

    void reset_callback(const std_msgs::msg::Empty::ConstSharedPtr)
    {
        reset_estimator_state();
        RCLCPP_INFO(get_logger(), "Homing estimator reset for reacquisition.");
    }

    void reset_estimator_state()
    {
        samples_.clear();
        snr_median_buffer_.clear();
        gradient_directions_.clear();
        particles_.clear();
        particles_initialized_ = false;
        particle_direction_prior_applied_ = false;
        last_particle_confidence_ = 0.0;
        have_filtered_direction_ = false;
        publish_estimator_status(false, false);
    }

    void prune_samples(const rclcpp::Time & stamp)
    {
        // 분기 A: 메모리와 표본 개수 제한을 적용한다.
        while (samples_.size() > max_sample_count_) {
            samples_.pop_front();
        }
        // 분기 B: 현재 국소 SNR장을 나타내기 어려운 오래된 표본을 제거한다.
        while (!samples_.empty() && (stamp - samples_.front().stamp).seconds() > max_sample_age_s_) {
            samples_.pop_front();
        }
    }

    void update_direction(const rclcpp::Time & stamp)
    {
        // 3단계: gradient 추정값을 필수 기본 관측으로 사용한다.
        Eigen::Vector3d gradient_direction;
        double gradient_confidence = 0.0;
        // 분기: 충분한 이동량과 SNR 변화가 확보될 때까지 발행하지 않는다.
        if (!estimate_gradient_direction(gradient_direction, gradient_confidence)) {
            publish_estimator_status(false, false);
            return;
        }

        gradient_directions_.push_back(gradient_direction);
        while (gradient_directions_.size() > gradient_stability_window_) {
            gradient_directions_.pop_front();
        }
        const bool gradient_stable = is_gradient_direction_stable();
        const bool estimator_ready =
            trajectory_coverage_ratio_ >= min_trajectory_coverage_ratio_ && gradient_stable;
        publish_estimator_status(estimator_ready, estimate_near_source());

        // 충분한 공간 표본으로 처음 얻은 gradient를 PF의 방향성 prior로 한 번 반영한다.
        if (enable_particle_filter_ && estimator_ready && !particle_direction_prior_applied_) {
            apply_particle_direction_prior(gradient_direction);
        }

        // 기본 분기: PF가 비활성/사용 불가하거나 direction_source가 "gradient"이면
        // gradient를 사용한다. 알 수 없는 설정값에 대해서도 gradient로 대체한다.
        Eigen::Vector3d direction = gradient_direction;
        double confidence = gradient_confidence;

        Eigen::Vector3d particle_direction;
        double particle_confidence = 0.0;
        // 4단계: PF는 선택 사항이며 gradient만으로도 동작할 수 있다.
        const bool have_particle_direction =
            enable_particle_filter_ && estimate_particle_direction(particle_direction, particle_confidence);
        if (have_particle_direction) {
            const double agreement = gradient_direction.dot(particle_direction);
            // 분기 A: 설정된 안전 조건에 따라 서로 모순되는 추정 결과를 거부한다.
            if (require_particle_agreement_ && agreement < particle_agreement_min_dot_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "SNR gradient/PF directions disagree: dot=%.2f. Direction not published.",
                    agreement);
                return;
            }
            // 분기 B: PF 평균 위치 방향만 최종 방향으로 사용한다.
            if (direction_source_ == "particle") {
                direction = particle_direction;
                confidence = particle_confidence;
            // 분기 C: 국소 gradient와 누적된 PF 정보를 융합한다.
            } else if (direction_source_ == "blend") {
                direction = (1.0 - particle_blend_) * gradient_direction + particle_blend_ * particle_direction;
                const double norm = direction.norm();
                // 서로 반대인 벡터가 상쇄되면 직접 추정한 gradient로 대체한다.
                if (norm < 1.0e-9) {
                    direction = gradient_direction;
                } else {
                    direction /= norm;
                }
                confidence = std::min(1.0, 0.65 * gradient_confidence + 0.35 * particle_confidence);
            }
        }

        // 5단계: 선택한 homing 방향을 평활화하여 발행한다.
        publish_direction(stamp, filter_direction(direction), confidence);
        // PF 음원 좌표는 particle이 생성된 이후에만 유효하다.
        if (have_particle_direction) {
            publish_source_estimate(stamp);
        }
    }

    bool estimate_gradient_direction(Eigen::Vector3d & direction, double & confidence)
    {
        // 관측 가능성 검사 A: 회귀 계산에 필요한 최소 표본 수를 확인한다.
        if (samples_.size() < min_sample_count_) {
            return false;
        }

        // 회귀가 공간 기울기만 추정하도록 위치와 SNR의 평균을 제거한다.
        Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();
        double mean_snr = 0.0;
        for (const Sample & sample : samples_) {
            mean_position += sample.position_m;
            mean_snr += sample.snr;
        }
        mean_position /= static_cast<double>(samples_.size());
        mean_snr /= static_cast<double>(samples_.size());

        // C_pp * gradient = C_pS를 풀어 최소제곱 국소 SNR 평면을 구한다.
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * covariance_regularization_;
        Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
        double min_snr = std::numeric_limits<double>::infinity();
        double max_snr = -std::numeric_limits<double>::infinity();
        double max_radius_m = 0.0;
        for (const Sample & sample : samples_) {
            Eigen::Vector3d delta_position = sample.position_m - mean_position;
            // 분기: 평면 AUV homing에서는 수심 변화를 무시한다.
            if (horizontal_only_) {
                delta_position.z() = 0.0;
            }
            const double delta_snr = sample.snr - mean_snr;
            covariance += delta_position * delta_position.transpose();
            rhs += delta_position * delta_snr;
            min_snr = std::min(min_snr, sample.snr);
            max_snr = std::max(max_snr, sample.snr);
            max_radius_m = std::max(max_radius_m, delta_position.norm());
        }

        const double motion_baseline_m = 2.0 * max_radius_m;
        const double snr_span = max_snr - min_snr;
        Eigen::Matrix2d horizontal_covariance = covariance.topLeftCorner<2, 2>();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(horizontal_covariance);
        if (eigen_solver.info() != Eigen::Success) {
            return false;
        }
        const double smallest = std::max(0.0, eigen_solver.eigenvalues().x());
        const double largest = std::max(1.0e-12, eigen_solver.eigenvalues().y());
        trajectory_coverage_ratio_ = std::clamp(smallest / largest, 0.0, 1.0);
        // 관측 가능성 검사 B: 이동량과 SNR 대비가 잡음 임계값보다 커야 한다.
        if (motion_baseline_m < min_motion_baseline_m_ || snr_span < min_snr_span_) {
            return false;
        }

        // 단조 SNR 모델에서는 양의 SNR gradient가 음원 방향이다.
        Eigen::Vector3d gradient = covariance.ldlt().solve(rhs);
        if (horizontal_only_) {
            gradient.z() = 0.0;
        }
        const double gradient_norm = gradient.norm();
        // 분기: 특이하거나 유한하지 않거나 사실상 평탄한 추정값은 거부한다.
        if (!std::isfinite(gradient_norm) || gradient_norm < 1.0e-9) {
            return false;
        }

        direction = gradient / gradient_norm;
        const double expected_span = gradient_norm * std::max(motion_baseline_m, 1.0e-6);
        confidence = std::clamp(expected_span / std::max(snr_span, min_snr_span_), 0.0, 1.0);
        return true;
    }

    bool is_gradient_direction_stable() const
    {
        if (gradient_directions_.size() < gradient_stability_window_) {
            return false;
        }
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (const Eigen::Vector3d & direction : gradient_directions_) {
            mean += direction.head<2>();
        }
        if (mean.norm() < 1.0e-9) {
            return false;
        }
        const double mean_angle = std::atan2(mean.y(), mean.x());
        double squared_error = 0.0;
        for (const Eigen::Vector3d & direction : gradient_directions_) {
            double error = std::atan2(direction.y(), direction.x()) - mean_angle;
            error = std::atan2(std::sin(error), std::cos(error));
            squared_error += error * error;
        }
        const double std_rad = std::sqrt(
            squared_error / static_cast<double>(gradient_directions_.size()));
        return std_rad <= max_gradient_std_rad_;
    }

    bool estimate_near_source() const
    {
        if (samples_.size() < 2 || !particles_initialized_) {
            return false;
        }
        Eigen::Vector3d cluster_center;
        double cluster_mass = 0.0;
        if (!estimate_dominant_particle_cluster(cluster_center, cluster_mass)) {
            return false;
        }
        const double distance = (cluster_center - current_position_m_).norm();
        const double snr_delta = std::abs(samples_.back().snr - samples_[samples_.size() - 2].snr);
        return samples_.back().snr >= near_source_min_snr_ &&
            distance <= near_source_distance_m_ && snr_delta <= near_source_max_snr_delta_;
    }

    void publish_estimator_status(const bool ready, const bool near_source)
    {
        std_msgs::msg::Bool ready_msg;
        ready_msg.data = ready;
        estimator_ready_pub_->publish(ready_msg);
        std_msgs::msg::Bool near_msg;
        near_msg.data = near_source;
        near_source_pub_->publish(near_msg);
    }

    void update_particles(const Sample & sample)
    {
        // 4-a단계: 최초 관측 위치 주변에 음원 후보 particle을 생성한다.
        if (!particles_initialized_) {
            initialize_particles(sample.position_m);
        }
        // 분기: 거리/SNR 변화 계산에는 현재 표본과 이전 표본이 모두 필요하다.
        if (samples_.empty() || particles_.empty()) {
            return;
        }

        const Sample & previous = samples_.back();
        const double delta_snr = sample.snr - previous.snr;
        // 분기: SNR 잡음과 구별하기 어려운 작은 변화는 무시한다.
        if (std::abs(delta_snr) <= snr_deadband_) {
            return;
        }

        const double vehicle_motion_m = (sample.position_m - previous.position_m).norm();
        if (vehicle_motion_m <= particle_motion_deadband_m_) {
            return;
        }

        // 4-b단계: 부호뿐 아니라 이동 투영량과 SNR 변화량도 likelihood에 반영한다.
        for (Particle & particle : particles_) {
            const double previous_range_m = (particle.position_m - previous.position_m).norm();
            const double current_range_m = (particle.position_m - sample.position_m).norm();
            const double delta_range_m = current_range_m - previous_range_m;
            // 분기: 거리 변화가 너무 작으면 이 이동은 해당 후보의 근거로 쓰지 않는다.
            if (std::abs(delta_range_m) <= particle_motion_deadband_m_) {
                continue;
            }
            const double range_evidence = std::clamp(
                -delta_range_m / std::max(vehicle_motion_m, 1.0e-9), -1.0, 1.0);
            const double snr_evidence = std::tanh(delta_snr / particle_snr_scale_);
            particle.weight *= std::exp(
                particle_update_gain_ * range_evidence * snr_evidence);
        }

        normalize_particle_weights();
        last_particle_confidence_ = std::clamp(
            1.0 - effective_particle_count() / static_cast<double>(particles_.size()),
            0.0,
            1.0);
        // 4-c단계: 소수 particle에 가중치가 집중되면 재표본화한다.
        if (effective_particle_count() <
            particle_resample_ess_ratio_ * static_cast<double>(particles_.size()))
        {
            resample_particles(sample.position_m);
        }
    }

    void initialize_particles(const Eigen::Vector3d & center_m)
    {
        // 면적 기준으로 균일한 원환 분포를 사용해 후보가 중심에 몰리지 않게 한다.
        particles_.clear();
        particles_.reserve(particle_count_);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        std::uniform_real_distribution<double> angle_dist(-PI, PI);
        for (std::size_t i = 0; i < particle_count_; ++i) {
            const double angle = angle_dist(rng_);
            const double radius = particle_radius_min_m_ +
                (particle_radius_max_m_ - particle_radius_min_m_) * std::sqrt(unit(rng_));
            Particle particle;
            particle.position_m = center_m + Eigen::Vector3d(
                radius * std::cos(angle),
                radius * std::sin(angle),
                0.0);
            particle.weight = 1.0 / static_cast<double>(particle_count_);
            particles_.push_back(particle);
        }
        particles_initialized_ = true;
    }

    void normalize_particle_weights()
    {
        const double weight_sum = std::accumulate(
            particles_.begin(),
            particles_.end(),
            0.0,
            [](const double sum, const Particle & particle) {
                return sum + particle.weight;
            });
        // 복구 분기: 유효하지 않거나 underflow된 가중치를 균일 사전분포로 초기화한다.
        if (weight_sum <= 0.0 || !std::isfinite(weight_sum)) {
            const double uniform_weight = 1.0 / static_cast<double>(particles_.size());
            for (Particle & particle : particles_) {
                particle.weight = uniform_weight;
            }
            return;
        }
        for (Particle & particle : particles_) {
            particle.weight /= weight_sum;
        }
    }

    double effective_particle_count() const
    {
        double squared_sum = 0.0;
        for (const Particle & particle : particles_) {
            squared_sum += particle.weight * particle.weight;
        }
        return squared_sum > 0.0 ? 1.0 / squared_sum : 0.0;
    }

    void resample_particles(const Eigen::Vector3d & current_position_m)
    {
        // systematic resampling으로 분산을 낮게 유지하며 가능성 높은 후보를 보존한다.
        std::vector<Particle> resampled;
        resampled.reserve(particles_.size());
        std::uniform_real_distribution<double> unit(0.0, 1.0 / static_cast<double>(particles_.size()));
        double cursor = unit(rng_);
        std::size_t index = 0;
        double cumulative = particles_.empty() ? 0.0 : particles_.front().weight;
        for (std::size_t i = 0; i < particles_.size(); ++i) {
            const double target = cursor + static_cast<double>(i) / static_cast<double>(particles_.size());
            while (target > cumulative && index + 1 < particles_.size()) {
                ++index;
                cumulative += particles_[index].weight;
            }
            Particle particle = particles_[index];
            particle.weight = 1.0 / static_cast<double>(particles_.size());
            // 복제된 particle에 작은 위치 잡음을 넣어 다양성 붕괴를 방지한다.
            if (particle_roughening_std_m_ > 0.0) {
                std::normal_distribution<double> noise(0.0, particle_roughening_std_m_);
                particle.position_m.x() += noise(rng_);
                particle.position_m.y() += noise(rng_);
            }
            // 분기: 축소된 particle이 최소 음원 반경 안으로 들어가지 않도록 한다.
            if ((particle.position_m - current_position_m).norm() < particle_radius_min_m_) {
                particle.position_m += random_horizontal_unit() * particle_radius_min_m_;
            }
            resampled.push_back(particle);
        }
        particles_ = std::move(resampled);
    }

    Eigen::Vector3d random_horizontal_unit()
    {
        std::uniform_real_distribution<double> angle_dist(-PI, PI);
        const double angle = angle_dist(rng_);
        return Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
    }

    void apply_particle_direction_prior(const Eigen::Vector3d & direction)
    {
        if (!particles_initialized_ || particles_.empty()) {
            return;
        }
        Eigen::Vector3d horizontal_direction = direction;
        horizontal_direction.z() = 0.0;
        const double norm = horizontal_direction.norm();
        if (norm < 1.0e-9) {
            return;
        }
        horizontal_direction /= norm;
        for (Particle & particle : particles_) {
            Eigen::Vector3d offset = particle.position_m - current_position_m_;
            offset.z() = 0.0;
            const double offset_norm = offset.norm();
            if (offset_norm > 1.0e-9) {
                particle.weight *= std::exp(
                    particle_prior_strength_ * horizontal_direction.dot(offset / offset_norm));
            }
        }
        normalize_particle_weights();
        particle_direction_prior_applied_ = true;
    }

    bool estimate_dominant_particle_cluster(
        Eigen::Vector3d & center, double & cluster_mass) const
    {
        if (particles_.empty()) {
            return false;
        }
        const double radius_sq = particle_cluster_radius_m_ * particle_cluster_radius_m_;
        std::size_t best_seed = 0;
        double best_mass = -1.0;
        for (std::size_t i = 0; i < particles_.size(); ++i) {
            double mass = 0.0;
            for (const Particle & candidate : particles_) {
                if ((candidate.position_m - particles_[i].position_m).squaredNorm() <= radius_sq) {
                    mass += candidate.weight;
                }
            }
            if (mass > best_mass) {
                best_mass = mass;
                best_seed = i;
            }
        }

        center = Eigen::Vector3d::Zero();
        cluster_mass = 0.0;
        for (const Particle & particle : particles_) {
            if ((particle.position_m - particles_[best_seed].position_m).squaredNorm() <= radius_sq) {
                center += particle.weight * particle.position_m;
                cluster_mass += particle.weight;
            }
        }
        if (cluster_mass <= 0.0) {
            return false;
        }
        center /= cluster_mass;
        return true;
    }

    bool estimate_particle_direction(Eigen::Vector3d & direction, double & confidence) const
    {
        // 분기: 사전분포가 초기화되기 전에는 PF 방향을 계산할 수 없다.
        if (!particles_initialized_ || particles_.empty()) {
            return false;
        }

        // 전체 평균 대신 가장 큰 국소 cluster를 사용해 다봉분포의 허위 중간점을 피한다.
        Eigen::Vector3d mean_source;
        double cluster_mass = 0.0;
        if (!estimate_dominant_particle_cluster(mean_source, cluster_mass)) {
            return false;
        }
        Eigen::Vector3d vector_to_source = mean_source - current_position_m_;
        if (horizontal_only_) {
            vector_to_source.z() = 0.0;
        }
        const double norm = vector_to_source.norm();
        // 분기: 음원 추정 위치가 기체와 같으면 방향을 정의할 수 없다.
        if (norm < 1.0e-9) {
            return false;
        }

        direction = vector_to_source / norm;
        // resampling 직후 ESS가 초기화되어도 직전 집중도와 cluster 질량을 보존한다.
        confidence = std::clamp(cluster_mass * (0.5 + 0.5 * last_particle_confidence_), 0.0, 1.0);
        return true;
    }

    Eigen::Vector3d filter_direction(const Eigen::Vector3d & direction)
    {
        // 초기화 분기: 최초 유효 방향으로 저역통과 필터를 초기화한다.
        if (!have_filtered_direction_) {
            filtered_direction_ = direction;
            have_filtered_direction_ = true;
            return filtered_direction_;
        }

        filtered_direction_ =
            (1.0 - direction_filter_alpha_) * filtered_direction_ + direction_filter_alpha_ * direction;
        const double norm = filtered_direction_.norm();
        // 분기: 급격한 방향 반전으로 상쇄된 벡터를 정규화하지 않는다.
        if (norm < 1.0e-9) {
            filtered_direction_ = direction;
        } else {
            filtered_direction_ /= norm;
        }
        return filtered_direction_;
    }

    void publish_direction(
        const rclcpp::Time & stamp,
        const Eigen::Vector3d & world_direction,
        const double confidence)
    {
        Eigen::Vector3d output_direction = world_direction;
        std::string frame_id = output_frame_id_.empty() ? odometry_frame_id_ : output_frame_id_;
        // 출력 분기: 기체 좌표계 제어기는 odometry 방향을 현재 yaw만큼 회전하고,
        // 월드 좌표계 제어기는 방향을 변환하지 않고 사용한다.
        if (output_frame_ == "base_link" || output_frame_ == "body") {
            output_direction = world_to_body_direction(world_direction);
            frame_id = output_frame_id_.empty() ? "base_link" : output_frame_id_;
        }

        geometry_msgs::msg::Vector3Stamped direction_msg;
        direction_msg.header.stamp = stamp;
        direction_msg.header.frame_id = frame_id;
        direction_msg.vector.x = output_direction.x();
        direction_msg.vector.y = output_direction.y();
        direction_msg.vector.z = output_direction.z();
        direction_pub_->publish(direction_msg);

        std_msgs::msg::Float64 confidence_msg;
        confidence_msg.data = std::clamp(confidence, 0.0, 1.0);
        confidence_pub_->publish(confidence_msg);
    }

    Eigen::Vector3d world_to_body_direction(const Eigen::Vector3d & world_direction) const
    {
        const double c = std::cos(current_yaw_rad_);
        const double s = std::sin(current_yaw_rad_);
        return Eigen::Vector3d(
            c * world_direction.x() + s * world_direction.y(),
            -s * world_direction.x() + c * world_direction.y(),
            world_direction.z());
    }

    void publish_source_estimate(const rclcpp::Time & stamp)
    {
        // 분기: 초기화되지 않은 음원 위치 추정값은 발행하지 않는다.
        if (!particles_initialized_ || particles_.empty()) {
            return;
        }
        Eigen::Vector3d mean_source;
        double cluster_mass = 0.0;
        if (!estimate_dominant_particle_cluster(mean_source, cluster_mass)) {
            return;
        }

        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = odometry_frame_id_;
        msg.point.x = mean_source.x();
        msg.point.y = mean_source.y();
        msg.point.z = mean_source.z();
        source_estimate_pub_->publish(msg);
    }

    static double yaw_from_quaternion(const double w, const double x, const double y, const double z)
    {
        const double siny_cosp = 2.0 * (w * z + x * y);
        const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    std::string snr_topic_;
    std::string odometry_topic_;
    std::string depth_topic_;
    std::string direction_topic_;
    std::string confidence_topic_;
    std::string source_estimate_topic_;
    std::string estimator_ready_topic_;
    std::string near_source_topic_;
    std::string reset_topic_;
    std::string output_frame_;
    std::string output_frame_id_;
    bool use_depth_pose_ = true;
    bool horizontal_only_ = true;

    std::size_t max_sample_count_ = 160;
    std::size_t min_sample_count_ = 16;
    double max_sample_age_s_ = 45.0;
    double min_sample_spacing_m_ = 0.03;
    double min_motion_baseline_m_ = 0.30;
    double min_snr_span_ = 0.20;
    double snr_deadband_ = 0.02;
    double direction_filter_alpha_ = 0.25;
    double covariance_regularization_ = 1.0e-4;
    double min_trajectory_coverage_ratio_ = 0.12;
    std::size_t gradient_stability_window_ = 8;
    double max_gradient_std_rad_ = 0.30;
    std::size_t snr_median_window_ = 5;
    double odometry_history_age_s_ = 10.0;
    double max_odometry_extrapolation_s_ = 0.10;
    double near_source_distance_m_ = 1.0;
    double near_source_min_snr_ = 5.0;
    double near_source_max_snr_delta_ = 0.10;

    bool enable_particle_filter_ = true;
    std::string direction_source_ = "blend";
    std::size_t particle_count_ = 500;
    double particle_radius_min_m_ = 0.5;
    double particle_radius_max_m_ = 15.0;
    double particle_update_gain_ = 0.8;
    double particle_resample_ess_ratio_ = 0.45;
    double particle_blend_ = 0.35;
    double particle_motion_deadband_m_ = 0.02;
    double particle_snr_scale_ = 0.5;
    double particle_roughening_std_m_ = 0.20;
    double particle_cluster_radius_m_ = 2.0;
    double particle_prior_strength_ = 2.0;
    bool require_particle_agreement_ = false;
    double particle_agreement_min_dot_ = 0.0;

    Eigen::Vector3d current_position_m_{0.0, 0.0, 0.0};
    double current_yaw_rad_ = 0.0;
    std::string odometry_frame_id_ = "odom";
    bool have_pose_ = false;
    bool have_depth_ = false;

    std::deque<Sample> samples_;
    std::deque<PoseSample> odometry_history_;
    std::deque<double> snr_median_buffer_;
    std::deque<Eigen::Vector3d> gradient_directions_;
    double trajectory_coverage_ratio_ = 0.0;
    std::vector<Particle> particles_;
    bool particles_initialized_ = false;
    bool particle_direction_prior_applied_ = false;
    double last_particle_confidence_ = 0.0;
    std::mt19937 rng_;
    Eigen::Vector3d filtered_direction_{1.0, 0.0, 0.0};
    bool have_filtered_direction_ = false;

    rclcpp::Subscription<audio_common_msgs::msg::Float64Stamped>::SharedPtr snr_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr confidence_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr source_estimate_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estimator_ready_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr near_source_pub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::SnrGradientHomingNode)
