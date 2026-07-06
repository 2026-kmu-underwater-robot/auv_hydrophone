#include <cstdint>

#include <cmath>
#include <complex>
#include <memory>
#include <vector>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>

namespace audio_capture
{
class AudioPhaseEstimatorNode : public rclcpp::Node
{
public:
    explicit AudioPhaseEstimatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("audio_phase_estimator", options)
    {
        audio_sub_ = this->create_subscription<audio_common_msgs::msg::AudioData>(
            "/audio",
            10,
            std::bind(&AudioPhaseEstimatorNode::audio_callback, this, std::placeholders::_1));
        // ===== CODEX MODIFIED START: DVL input and homing yaw output =====
        dvl_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered",
            10,
            std::bind(&AudioPhaseEstimatorNode::dvl_odometry_callback, this, std::placeholders::_1));
        dvl_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/dvl/twist",
            10,
            std::bind(&AudioPhaseEstimatorNode::dvl_twist_callback, this, std::placeholders::_1));
        homing_direction_pub_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/homing/direction", 10);
        yaw_reference_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/homing/yaw_reference_rad", 10);
        // ===== CODEX MODIFIED END: DVL input and homing yaw output =====

        worker_thread_ = std::thread(&AudioPhaseEstimatorNode::analysis_loop, this);
    }

    ~AudioPhaseEstimatorNode()
    {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            stop_worker_ = true;
        }
        buffer_cv_.notify_one();

        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

private:
    // ===== CODEX MODIFIED START: EKF estimator design =====
    // 상태 u=[u_x,u_y]^T, 관측 z=Delta r=-u^T Delta p.
    class HomingDirectionEkf
    {
    public:
        bool update(const Eigen::Vector2d & delta_position_m, const double delta_range_m)
        {
            if (delta_position_m.squaredNorm() < min_motion_squared_m2_) {
                return false;
            }

            covariance_ += process_noise_;

            const Eigen::RowVector2d measurement_jacobian = -delta_position_m.transpose();
            const double innovation =
                delta_range_m - static_cast<double>(measurement_jacobian * direction_);
            const double innovation_covariance =
                static_cast<double>(measurement_jacobian * covariance_ * measurement_jacobian.transpose()) +
                range_noise_variance_m2_;
            const Eigen::Vector2d kalman_gain =
                covariance_ * measurement_jacobian.transpose() / innovation_covariance;

            direction_ += kalman_gain * innovation;
            covariance_ =
                (Eigen::Matrix2d::Identity() - kalman_gain * measurement_jacobian) * covariance_;
            return true;
        }

        Eigen::Vector2d normalized_direction() const
        {
            const double norm = direction_.norm();
            if (norm < min_direction_norm_) {
                return Eigen::Vector2d::Zero();
            }
            return direction_ / norm;
        }

    private:
        Eigen::Vector2d direction_{1.0, 0.0};
        Eigen::Matrix2d covariance_ = Eigen::Matrix2d::Identity() * 10.0;
        Eigen::Matrix2d process_noise_ = Eigen::Matrix2d::Identity() * 1.0e-4;
        double range_noise_variance_m2_ = 1.0e-3;
        double min_motion_squared_m2_ = 1.0e-6;
        double min_direction_norm_ = 1.0e-9;
    };
    // ===== CODEX MODIFIED END: EKF estimator design =====

    void audio_callback(const audio_common_msgs::msg::AudioData::ConstSharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            append_samples_from_pcm(msg->data);
        }
        buffer_cv_.notify_one();
    }

    // ===== CODEX MODIFIED START: DVL deltaP accumulation =====
    void dvl_odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        const Eigen::Vector2d position_m(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y);

        std::lock_guard<std::mutex> lock(dvl_mutex_);
        if (have_previous_dvl_position_) {
            accumulated_delta_position_m_ += position_m - previous_dvl_position_m_;
        }
        previous_dvl_position_m_ = position_m;
        have_previous_dvl_position_ = true;
        have_dvl_odometry_ = true;
    }

    void dvl_twist_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(dvl_mutex_);
        if (have_dvl_odometry_) {
            return;
        }

        const rclcpp::Time stamp(msg->header.stamp);
        if (have_previous_dvl_twist_stamp_) {
            const double dt = (stamp - previous_dvl_twist_stamp_).seconds();
            if (dt > 0.0 && dt < max_twist_dt_s_) {
                accumulated_delta_position_m_ += Eigen::Vector2d(
                    msg->twist.linear.x * dt,
                    msg->twist.linear.y * dt);
            }
        }
        previous_dvl_twist_stamp_ = stamp;
        have_previous_dvl_twist_stamp_ = true;
    }

    Eigen::Vector2d consume_accumulated_delta_position()
    {
        std::lock_guard<std::mutex> lock(dvl_mutex_);
        const Eigen::Vector2d delta_position_m = accumulated_delta_position_m_;
        accumulated_delta_position_m_.setZero();
        return delta_position_m;
    }
    // ===== CODEX MODIFIED END: DVL deltaP accumulation =====

    void analysis_loop()
    {
        while (rclcpp::ok()) {
            std::vector<double> window;
            std::uint64_t window_start_sample = 0;

            {
                std::unique_lock<std::mutex> lock(buffer_mutex_);
                buffer_cv_.wait(lock, [this]() {
                    return stop_worker_ || sample_buffer_.size() >= window_size_;
                });

                if (stop_worker_) {
                    break;
                }

                window.assign(sample_buffer_.begin(), sample_buffer_.begin() + window_size_);
                sample_buffer_.erase(sample_buffer_.begin(), sample_buffer_.begin() + window_size_);
                window_start_sample = next_window_start_sample_;
                next_window_start_sample_ += window_size_;
            }

            analyze_window(window, window_start_sample);
        }
    }

    void analyze_window(const std::vector<double> & window, const std::uint64_t window_start_sample)
    {
        // 대회에서 알려진 기준 주파수로 복조해 baseband 복소값 z_k = I + jQ를 얻는다.
        const std::complex<double> iq = demodulate_iq(window, window_start_sample, reference_frequency_hz_);//Z_k = x[n] * exp(-j 2*pi*f_ref*t)
        if (std::abs(iq) < min_iq_magnitude_) {
            have_previous_iq_ = false;
            return;
        }

        const double phase_rad = std::atan2(std::imag(iq), std::real(iq));  //theta_k = arg(Z_k)
        double delta_phase_rad = 0.0;  //delta_theta_k = theta_k - theta_k-1
        // ===== CODEX MODIFIED START: phase difference to range difference =====
        double delta_range_m = 0.0; // Delta r = -lambda * Delta theta / (2*pi)
        if (have_previous_iq_) {  //theta_k-1가 있으면
            // 두 window의 켤레곱을 쓰면 -pi~pi 범위의 안정적인 위상차를 바로 얻을 수 있다.
            const std::complex<double> phase_step = iq * std::conj(previous_iq_);//Z_k * Z_k-1^*
            delta_phase_rad = std::atan2(std::imag(phase_step), std::real(phase_step));
            delta_range_m = -wavelength_m_ * delta_phase_rad / (2.0 * M_PI);    //47.2ms당 거리 변화량
            update_homing_estimate(delta_range_m);
        }
        // ===== CODEX MODIFIED END: phase difference to range difference =====

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "phase estimator f_ref: %.1f Hz, |z|: %.6f, phase: %.4f rad, dphase: %.4f rad, dr: %.6f m",
            reference_frequency_hz_,
            std::abs(iq),
            phase_rad,
            delta_phase_rad,
            delta_range_m);

        previous_iq_ = iq;
        have_previous_iq_ = true;
    }

    std::complex<double> demodulate_iq(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const double frequency_hz) const
    {
        std::complex<double> baseband_sum(0.0, 0.0);
        const double phase_step = 2.0 * M_PI * frequency_hz / static_cast<double>(sampling_rate_);

        for (std::size_t n = 0; n < window.size(); ++n) {
            // 1) I/Q 복조: x[n] * exp(-j 2*pi*f_ref*t)로 carrier를 baseband로 내린다.
            const double phase = phase_step * static_cast<double>(window_start_sample + n);
            const std::complex<double> mixed_sample =
                window[n] * std::complex<double>(std::cos(phase), -std::sin(phase));
            baseband_sum += mixed_sample;
        }

        // 2) LPF: 한 window 동안의 baseband 평균을 내서 2*f_ref 성분과 빠른 흔들림을 제거한다.
        return baseband_sum / static_cast<double>(window.size());
    }

    void append_samples_from_pcm(const std::vector<uint8_t> & data)
    {
        // /audio는 S32LE 2채널 interleaved PCM이므로 선택한 채널만 double 샘플로 변환한다.
        const std::size_t channel_offset = channel_index_ * bytes_per_sample_;
        for (std::size_t frame_start = 0; frame_start + frame_size_ <= data.size();
            frame_start += frame_size_)
        {
            const int32_t sample = read_int32_little_endian(data, frame_start + channel_offset);
            sample_buffer_.push_back(static_cast<double>(sample) / 2147483648.0);
        }
    }

    int32_t read_int32_little_endian(const std::vector<uint8_t> & data, const std::size_t offset) const
    {
        const uint32_t raw =
            static_cast<uint32_t>(data[offset]) |
            (static_cast<uint32_t>(data[offset + 1]) << 8) |
            (static_cast<uint32_t>(data[offset + 2]) << 16) |
            (static_cast<uint32_t>(data[offset + 3]) << 24);
        return static_cast<int32_t>(raw);
    }

    // ===== CODEX MODIFIED START: EKF output to yaw reference =====
    void update_homing_estimate(const double delta_range_m)
    {
        const Eigen::Vector2d delta_position_m = consume_accumulated_delta_position();
        if (!homing_direction_ekf_.update(delta_position_m, delta_range_m)) {
            return;
        }

        const Eigen::Vector2d direction = homing_direction_ekf_.normalized_direction();
        if (direction.isZero()) {
            return;
        }

        geometry_msgs::msg::Vector3Stamped direction_msg;
        direction_msg.header.stamp = this->now();
        direction_msg.header.frame_id = "dvl";
        direction_msg.vector.x = direction.x();
        direction_msg.vector.y = direction.y();
        direction_msg.vector.z = 0.0;
        homing_direction_pub_->publish(direction_msg);

        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = std::atan2(direction.y(), direction.x());
        yaw_reference_pub_->publish(yaw_msg);
    }
    // ===== CODEX MODIFIED END: EKF output to yaw reference =====

    rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;
    // ===== CODEX MODIFIED START: DVL I/O members =====
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr homing_direction_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_reference_pub_;
    // ===== CODEX MODIFIED END: DVL I/O members =====

    std::vector<double> sample_buffer_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::thread worker_thread_;
    bool stop_worker_ = false;

    std::size_t channels_ = 2;
    std::size_t channel_index_ = 0;
    std::size_t bytes_per_sample_ = 4;
    std::size_t frame_size_ = channels_ * bytes_per_sample_;

    std::size_t window_size_ = 4096;
    std::uint64_t next_window_start_sample_ = 0;
    std::size_t sampling_rate_ = 96000;
    double reference_frequency_hz_ = 21164.0; //27211.0도 있을 수 있음.
    // ===== CODEX MODIFIED START: acoustic phase constants =====
    double sound_speed_mps_ = 1500.0; // 수조/해역에 맞춰 보정할 음속.
    double wavelength_m_ = sound_speed_mps_ / reference_frequency_hz_;
    // ===== CODEX MODIFIED END: acoustic phase constants =====
    double min_iq_magnitude_ = 1.0e-8;

    bool have_previous_iq_ = false;
    std::complex<double> previous_iq_{0.0, 0.0};
    HomingDirectionEkf homing_direction_ekf_; // CODEX MODIFIED: DVL Delta p 연결 시 update() 호출.
    // ===== CODEX MODIFIED START: DVL state =====
    std::mutex dvl_mutex_;
    Eigen::Vector2d accumulated_delta_position_m_{0.0, 0.0};
    Eigen::Vector2d previous_dvl_position_m_{0.0, 0.0};
    rclcpp::Time previous_dvl_twist_stamp_;
    bool have_previous_dvl_position_ = false;
    bool have_previous_dvl_twist_stamp_ = false;
    bool have_dvl_odometry_ = false;
    double max_twist_dt_s_ = 0.5;
    // ===== CODEX MODIFIED END: DVL state =====
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::AudioPhaseEstimatorNode)
