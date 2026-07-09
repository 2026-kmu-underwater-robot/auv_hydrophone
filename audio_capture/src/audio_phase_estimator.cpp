#include <cstdint>

#include <algorithm>
#include <cmath>
#include <complex>
#include <deque>
#include <memory>
#include <vector>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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
        dvl_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered",
            10,
            std::bind(&AudioPhaseEstimatorNode::dvl_odometry_callback, this, std::placeholders::_1));
        depth_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/depth/pose",
            10,
            std::bind(&AudioPhaseEstimatorNode::depth_pose_callback, this, std::placeholders::_1));
        homing_direction_pub_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/homing/direction", 10);
        demodulation_frequency_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/audio_phase_estimator/demodulation_frequency_hz", 10);
        iq_snr_ratio_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/audio_phase_estimator/iq_snr_ratio", 10);
        iq_coherence_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/audio_phase_estimator/iq_coherence", 10);
        reference_frequency_hz_ =
            this->declare_parameter<double>("reference_frequency_hz", reference_frequency_hz_);
        demodulation_frequency_hz_ = this->declare_parameter<double>(
            "initial_demodulation_frequency_hz",
            reference_frequency_hz_);
        sound_speed_mps_ = this->declare_parameter<double>("sound_speed_mps", sound_speed_mps_);
        const int window_size_param = static_cast<int>(
            this->declare_parameter<int>("window_size", static_cast<int>(window_size_)));
        window_size_ = static_cast<std::size_t>(std::max(1, window_size_param));
        const int hop_size_param = static_cast<int>(
            this->declare_parameter<int>("hop_size", static_cast<int>(hop_size_)));
        hop_size_ = static_cast<std::size_t>(std::max(1, hop_size_param));
        hop_size_ = std::min(hop_size_, window_size_);
        sync_delay_s_ = this->declare_parameter<double>("sync_delay_s", 0.10);
        min_iq_magnitude_ = this->declare_parameter<double>("min_iq_magnitude", 1.0e-5);
        min_iq_snr_ratio_ = this->declare_parameter<double>("min_iq_snr_ratio", min_iq_snr_ratio_);
        min_iq_coherence_ = this->declare_parameter<double>("min_iq_coherence", min_iq_coherence_);
        const int coherence_segments_param = static_cast<int>(
            this->declare_parameter<int>("coherence_segments", static_cast<int>(coherence_segments_)));
        coherence_segments_ = static_cast<std::size_t>(std::max(1, coherence_segments_param));
        direction_filter_alpha_ = this->declare_parameter<double>("direction_filter_alpha", 0.12);
        enable_frequency_acquisition_ =
            this->declare_parameter<bool>("enable_frequency_acquisition", enable_frequency_acquisition_);
        frequency_search_half_width_hz_ = this->declare_parameter<double>(
            "frequency_search_half_width_hz",
            frequency_search_half_width_hz_);
        frequency_search_step_hz_ = this->declare_parameter<double>(
            "frequency_search_step_hz",
            frequency_search_step_hz_);
        frequency_reacquire_threshold_hz_ = this->declare_parameter<double>(
            "frequency_reacquire_threshold_hz",
            frequency_reacquire_threshold_hz_);
        const int frequency_lock_required_windows_param = static_cast<int>(
            this->declare_parameter<int>(
                "frequency_lock_required_windows",
                frequency_lock_required_windows_));
        frequency_lock_required_windows_ = std::max(1, frequency_lock_required_windows_param);
        frequency_lock_tolerance_hz_ = this->declare_parameter<double>(
            "frequency_lock_tolerance_hz",
            frequency_lock_tolerance_hz_);

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
    // 상태 x=[u_x,u_y,u_z,b]^T, 관측 z=Delta r=-u^T Delta p + b*Delta t.
    // b는 단일 하이드로폰에서 주파수/클럭 drift가 거리 변화처럼 보이는 bias range-rate[m/s]이다.
    class HomingDirectionEkf
    {
    public:
        bool update(
            const Eigen::Vector3d & delta_position_m,
            const double delta_range_m,
            const double delta_time_s)
        {
            if (delta_time_s <= 0.0) {
                return false;
            }

            covariance_ += process_noise_;

            Eigen::Matrix<double, 1, 4> measurement_jacobian;
            measurement_jacobian << -delta_position_m.x(), -delta_position_m.y(), -delta_position_m.z(), delta_time_s;
            const double innovation =
                delta_range_m - static_cast<double>(measurement_jacobian * state_);
            const double innovation_covariance =
                static_cast<double>(measurement_jacobian * covariance_ * measurement_jacobian.transpose()) +
                range_noise_variance_m2_;
            if (innovation_covariance <= 1.0e-12) {
                return false;
            }

            const Eigen::Matrix<double, 4, 1> kalman_gain =
                covariance_ * measurement_jacobian.transpose() / innovation_covariance;

            state_ += kalman_gain * innovation;
            covariance_ =
                (Eigen::Matrix4d::Identity() - kalman_gain * measurement_jacobian) * covariance_;
            normalize_direction_state();
            return delta_position_m.squaredNorm() >= min_motion_squared_m2_;
        }

        Eigen::Vector3d normalized_direction() const
        {
            const Eigen::Vector3d direction = state_.head<3>();
            const double norm = direction.norm();
            if (norm < min_direction_norm_) {
                return Eigen::Vector3d::Zero();
            }
            return direction / norm;
        }

    private:
        void normalize_direction_state()
        {
            Eigen::Vector3d direction = state_.head<3>();
            const double norm = direction.norm();
            if (norm < min_direction_norm_) {
                return;
            }
            state_.head<3>() = direction / norm;
        }

        Eigen::Matrix<double, 4, 1> state_{1.0, 0.0, 0.0, 0.0};
        Eigen::Matrix4d covariance_ = Eigen::Matrix4d::Identity() * 10.0;
        Eigen::Matrix4d process_noise_ = []() {
            Eigen::Matrix4d noise = Eigen::Matrix4d::Identity() * 1.0e-4;
            noise(3, 3) = 1.0e-3;
            return noise;
        }();
        double range_noise_variance_m2_ = 1.0e-3;
        double min_motion_squared_m2_ = 1.0e-6;
        double min_direction_norm_ = 1.0e-9;
    };
    struct TimedSample
    {
        double value = 0.0;
        rclcpp::Time stamp;
    };

    struct TimedVector2
    {
        rclcpp::Time stamp;
        Eigen::Vector2d value{0.0, 0.0};
    };

    struct TimedScalar
    {
        rclcpp::Time stamp;
        double value = 0.0;
    };

    struct IqQuality
    {
        double magnitude = 0.0;
        double noise_magnitude = 0.0;
        double snr_ratio = 0.0;
        double coherence = 0.0;
    };

    void audio_callback(const audio_common_msgs::msg::AudioData::ConstSharedPtr msg)
    {
        const rclcpp::Time buffer_start_stamp = estimate_audio_buffer_start_stamp(msg->data.size());
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            append_samples_from_pcm(msg->data, buffer_start_stamp);
        }
        buffer_cv_.notify_one();
    }

    void dvl_odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(dvl_mutex_);
        odometry_buffer_.push_back({
            this->now(),
            Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y)});
        trim_old_samples(odometry_buffer_);
    }

    void depth_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(dvl_mutex_);
        depth_buffer_.push_back({this->now(), msg->pose.pose.position.z});
        trim_old_samples(depth_buffer_);
    }

    bool interpolate_position(const rclcpp::Time & stamp, Eigen::Vector3d & position_m) const
    {
        Eigen::Vector2d xy;
        double z = 0.0;
        if (!interpolate_xy(stamp, xy) || !hold_depth(stamp, z)) {
            return false;
        }
        position_m = Eigen::Vector3d(xy.x(), xy.y(), z);
        return true;
    }

    bool interpolate_xy(const rclcpp::Time & stamp, Eigen::Vector2d & value) const
    {
        if (odometry_buffer_.size() < 2 ||
            stamp < odometry_buffer_.front().stamp ||
            stamp > odometry_buffer_.back().stamp)
        {
            return false;
        }

        for (std::size_t i = 1; i < odometry_buffer_.size(); ++i) {
            if (stamp <= odometry_buffer_[i].stamp) {
                const double dt = (odometry_buffer_[i].stamp - odometry_buffer_[i - 1].stamp).seconds();
                if (dt <= 0.0) {
                    return false;
                }
                const double alpha = (stamp - odometry_buffer_[i - 1].stamp).seconds() / dt;
                value = odometry_buffer_[i - 1].value +
                    alpha * (odometry_buffer_[i].value - odometry_buffer_[i - 1].value);
                return true;
            }
        }
        return false;
    }

    bool hold_depth(const rclcpp::Time & stamp, double & value) const
    {
        if (depth_buffer_.empty() || stamp < depth_buffer_.front().stamp) {
            return false;
        }

        value = depth_buffer_.front().value;
        for (const auto & sample : depth_buffer_) {
            if (sample.stamp > stamp) {
                break;
            }
            value = sample.value;
        }
        return true;
    }

    template<typename SampleT>
    void trim_old_samples(std::deque<SampleT> & buffer) const
    {
        while (buffer.size() > max_pose_buffer_size_) {
            buffer.pop_front();
        }
    }
    void analysis_loop()
    {
        while (rclcpp::ok()) {
            std::vector<double> window;
            rclcpp::Time window_start_stamp;
            std::uint64_t window_start_sample = 0;

            {
                std::unique_lock<std::mutex> lock(buffer_mutex_);
                buffer_cv_.wait(lock, [this]() {
                    return stop_worker_ || sample_buffer_.size() >= window_size_ + sync_delay_samples();
                });

                if (stop_worker_) {
                    break;
                }

                window.reserve(window_size_);
                window_start_stamp = sample_buffer_.front().stamp;
                for (std::size_t i = 0; i < window_size_; ++i) {
                    window.push_back(sample_buffer_[i].value);
                }
                sample_buffer_.erase(sample_buffer_.begin(), sample_buffer_.begin() + hop_size_);
                window_start_sample = next_window_start_sample_;
                next_window_start_sample_ += hop_size_;
            }

            analyze_window(window, window_start_sample, window_start_stamp);
        }
    }

    std::size_t sync_delay_samples() const
    {
        return static_cast<std::size_t>(
            std::ceil(std::max(sync_delay_s_, 0.0) * static_cast<double>(sampling_rate_)));
    }

    void analyze_window(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const rclcpp::Time & window_start_stamp)
    {
        const auto window_duration = rclcpp::Duration::from_seconds(
            static_cast<double>(window.size()) / static_cast<double>(sampling_rate_));
        const rclcpp::Time window_center_stamp =
            window_start_stamp + rclcpp::Duration::from_seconds(0.5 * window_duration.seconds());

        update_demodulation_frequency(window, window_start_sample);
        if (enable_frequency_acquisition_ && !have_frequency_lock_) {
            have_previous_iq_ = false;
            return;
        }

        // coarse lock된 주파수로 복조해 baseband 복소값 z_k = I + jQ를 얻는다.
        const std::complex<double> iq = demodulate_iq(window, window_start_sample, demodulation_frequency_hz_);//Z_k = x[n] * exp(-j 2*pi*f_demod*t)
        const IqQuality iq_quality =
            estimate_iq_quality(window, window_start_sample, demodulation_frequency_hz_, iq);
        publish_iq_quality_debug(iq_quality);
        if (iq_quality.magnitude < min_iq_magnitude_) {
            have_previous_iq_ = false;
            return;
        }
        if (iq_quality.snr_ratio < min_iq_snr_ratio_ || iq_quality.coherence < min_iq_coherence_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "phase update skipped: weak IQ quality, |z| %.6f, snr %.2f, coherence %.2f",
                iq_quality.magnitude,
                iq_quality.snr_ratio,
                iq_quality.coherence);
            have_previous_iq_ = false;
            return;
        }

        double delta_phase_rad = 0.0;  //delta_theta_k = theta_k - theta_k-1
        double delta_range_m = 0.0; // Delta r = -lambda * Delta theta / (2*pi)
        if (have_previous_iq_) {  //theta_k-1가 있으면
            double delta_time_s = (window_center_stamp - previous_iq_stamp_).seconds();
            if (delta_time_s <= 0.0) {
                delta_time_s = window_duration.seconds();
            }

            // 두 window의 켤레곱을 쓰면 -pi~pi 범위의 안정적인 위상차를 바로 얻을 수 있다.
            const std::complex<double> phase_step = iq * std::conj(previous_iq_);//Z_k * Z_k-1^*
            delta_phase_rad = std::atan2(std::imag(phase_step), std::real(phase_step));
            delta_range_m = -current_wavelength_m() * delta_phase_rad / (2.0 * M_PI);
            update_homing_estimate(
                delta_range_m,
                delta_time_s,
                previous_iq_stamp_,
                window_center_stamp);
        }
        previous_iq_ = iq;
        previous_iq_stamp_ = window_center_stamp;
        have_previous_iq_ = true;
    }

    std::complex<double> demodulate_iq(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const double frequency_hz) const
    {
        std::complex<double> baseband_sum(0.0, 0.0);
        double weight_sum = 0.0;
        const double phase_step = 2.0 * M_PI * frequency_hz / static_cast<double>(sampling_rate_);
        const double hann_denominator = static_cast<double>(std::max<std::size_t>(1, window.size() - 1));

        for (std::size_t n = 0; n < window.size(); ++n) {
            const double weight =
                0.5 * (1.0 - std::cos((2.0 * M_PI * static_cast<double>(n)) / hann_denominator));
            // 1) I/Q 복조: x[n] * exp(-j 2*pi*f_ref*t)로 carrier를 baseband로 내린다.
            const double phase = phase_step * static_cast<double>(window_start_sample + n);
            const std::complex<double> mixed_sample =
                weight * window[n] * std::complex<double>(std::cos(phase), -std::sin(phase));
            baseband_sum += mixed_sample;
            weight_sum += weight;
        }

        // 2) LPF: 한 window 동안의 baseband 평균을 내서 2*f_ref 성분과 빠른 흔들림을 제거한다.
        return baseband_sum / std::max(weight_sum, 1.0e-12);
    }

    std::complex<double> demodulate_iq_segment(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const std::size_t offset,
        const std::size_t count,
        const double frequency_hz) const
    {
        std::complex<double> baseband_sum(0.0, 0.0);
        const double phase_step = 2.0 * M_PI * frequency_hz / static_cast<double>(sampling_rate_);
        const std::size_t end = std::min(window.size(), offset + count);
        for (std::size_t n = offset; n < end; ++n) {
            const double phase = phase_step * static_cast<double>(window_start_sample + n);
            baseband_sum += window[n] * std::complex<double>(std::cos(phase), -std::sin(phase));
        }
        const std::size_t used_count = end > offset ? end - offset : 0;
        if (used_count == 0) {
            return {0.0, 0.0};
        }
        return baseband_sum / static_cast<double>(used_count);
    }

    double estimate_iq_coherence(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const double frequency_hz) const
    {
        const std::size_t segments = std::max<std::size_t>(1, std::min(coherence_segments_, window.size()));
        const std::size_t segment_size = std::max<std::size_t>(1, window.size() / segments);
        std::complex<double> vector_sum(0.0, 0.0);
        double magnitude_sum = 0.0;

        for (std::size_t segment = 0; segment < segments; ++segment) {
            const std::size_t offset = segment * segment_size;
            if (offset >= window.size()) {
                break;
            }
            const std::size_t count = segment == segments - 1 ? window.size() - offset : segment_size;
            const std::complex<double> segment_iq =
                demodulate_iq_segment(window, window_start_sample, offset, count, frequency_hz);
            vector_sum += segment_iq;
            magnitude_sum += std::abs(segment_iq);
        }

        if (magnitude_sum <= 1.0e-12) {
            return 0.0;
        }
        return std::clamp(std::abs(vector_sum) / magnitude_sum, 0.0, 1.0);
    }

    IqQuality estimate_iq_quality(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const double frequency_hz,
        const std::complex<double> & target_iq) const
    {
        IqQuality quality;
        quality.magnitude = std::abs(target_iq);

        std::vector<double> noise_magnitudes;
        noise_magnitudes.reserve(6);
        const double offsets_hz[] = {-700.0, -450.0, -250.0, 250.0, 450.0, 700.0};
        for (const double offset_hz : offsets_hz) {
            const double probe_frequency_hz = frequency_hz + offset_hz;
            if (probe_frequency_hz <= 1.0) {
                continue;
            }
            noise_magnitudes.push_back(
                std::abs(demodulate_iq(window, window_start_sample, probe_frequency_hz)));
        }

        if (!noise_magnitudes.empty()) {
            std::sort(noise_magnitudes.begin(), noise_magnitudes.end());
            quality.noise_magnitude = noise_magnitudes[noise_magnitudes.size() / 2];
        }
        quality.snr_ratio = quality.magnitude / std::max(quality.noise_magnitude, 1.0e-12);
        quality.coherence = estimate_iq_coherence(window, window_start_sample, frequency_hz);
        return quality;
    }

    double current_wavelength_m() const
    {
        return sound_speed_mps_ / std::max(demodulation_frequency_hz_, 1.0);
    }

    void update_demodulation_frequency(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample)
    {
        if (!enable_frequency_acquisition_ || have_frequency_lock_) {
            return;
        }

        double acquired_frequency_hz = demodulation_frequency_hz_;
        if (!estimate_peak_frequency_hz(window, window_start_sample, acquired_frequency_hz)) {
            return;
        }

        if (pending_frequency_count_ == 0 ||
            std::abs(acquired_frequency_hz - pending_frequency_hz_) > frequency_lock_tolerance_hz_)
        {
            pending_frequency_hz_ = acquired_frequency_hz;
            pending_frequency_count_ = 1;
        } else {
            ++pending_frequency_count_;
        }

        if (pending_frequency_count_ < frequency_lock_required_windows_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "frequency lock pending: %.3f Hz (%d/%d)",
                pending_frequency_hz_,
                pending_frequency_count_,
                frequency_lock_required_windows_);
            return;
        }

        acquired_frequency_hz = pending_frequency_hz_;
        const double frequency_step_hz = acquired_frequency_hz - demodulation_frequency_hz_;
        if (std::abs(frequency_step_hz) < frequency_reacquire_threshold_hz_) {
            demodulation_frequency_hz_ = acquired_frequency_hz;
            have_frequency_lock_ = true;
            publish_demodulation_frequency_debug();
            RCLCPP_WARN(
                this->get_logger(),
                "Acquired demodulation frequency %.3f Hz.",
                demodulation_frequency_hz_);
            return;
        }

        demodulation_frequency_hz_ = acquired_frequency_hz;
        have_frequency_lock_ = true;
        have_previous_iq_ = false;
        publish_demodulation_frequency_debug();
        RCLCPP_WARN(
            this->get_logger(),
            "Acquired demodulation frequency %.3f Hz; phase tracking reset.",
            demodulation_frequency_hz_);
    }

    bool estimate_peak_frequency_hz(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        double & peak_frequency_hz)
    {
        if (frequency_search_half_width_hz_ <= 0.0 || frequency_search_step_hz_ <= 0.0) {
            return false;
        }

        const double search_start_hz =
            std::max(1.0, reference_frequency_hz_ - frequency_search_half_width_hz_);
        const double search_end_hz = reference_frequency_hz_ + frequency_search_half_width_hz_;
        double best_magnitude = -1.0;
        double best_frequency_hz = reference_frequency_hz_;

        for (double frequency_hz = search_start_hz;
            frequency_hz <= search_end_hz + 0.5 * frequency_search_step_hz_;
            frequency_hz += frequency_search_step_hz_)
        {
            const double magnitude = std::abs(demodulate_iq(window, window_start_sample, frequency_hz));
            if (magnitude > best_magnitude) {
                best_magnitude = magnitude;
                best_frequency_hz = frequency_hz;
            }
        }

        if (best_magnitude < min_iq_magnitude_) {
            return false;
        }

        const std::complex<double> best_iq =
            demodulate_iq(window, window_start_sample, best_frequency_hz);
        const IqQuality quality =
            estimate_iq_quality(window, window_start_sample, best_frequency_hz, best_iq);
        if (quality.snr_ratio < min_iq_snr_ratio_ || quality.coherence < min_iq_coherence_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "frequency candidate rejected: %.3f Hz, |z| %.6f, snr %.2f, coherence %.2f",
                best_frequency_hz,
                quality.magnitude,
                quality.snr_ratio,
                quality.coherence);
            return false;
        }

        peak_frequency_hz = best_frequency_hz;
        return true;
    }

    void append_samples_from_pcm(const std::vector<uint8_t> & data, const rclcpp::Time & buffer_start_stamp)
    {
        // /audio는 S32LE 2채널 interleaved PCM이므로 선택한 채널만 double 샘플로 변환한다.
        const std::size_t channel_offset = channel_index_ * bytes_per_sample_;
        std::uint64_t frame_index = 0;
        for (std::size_t frame_start = 0; frame_start + frame_size_ <= data.size();
            frame_start += frame_size_)
        {
            const int32_t sample = read_int32_little_endian(data, frame_start + channel_offset);
            const auto sample_offset = rclcpp::Duration::from_seconds(
                static_cast<double>(frame_index) / static_cast<double>(sampling_rate_));
            sample_buffer_.push_back({
                static_cast<double>(sample) / 2147483648.0,
                buffer_start_stamp + sample_offset});
            ++frame_index;
        }
    }

    rclcpp::Time estimate_audio_buffer_start_stamp(const std::size_t byte_count)
    {
        const std::size_t frame_count = byte_count / frame_size_;
        const auto buffer_duration = rclcpp::Duration::from_seconds(
            static_cast<double>(frame_count) / static_cast<double>(sampling_rate_));
        return this->now() - buffer_duration;
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

    void publish_iq_quality_debug(const IqQuality & iq_quality)
    {
        std_msgs::msg::Float64 snr_ratio_msg;
        snr_ratio_msg.data = iq_quality.snr_ratio;
        iq_snr_ratio_pub_->publish(snr_ratio_msg);

        std_msgs::msg::Float64 coherence_msg;
        coherence_msg.data = iq_quality.coherence;
        iq_coherence_pub_->publish(coherence_msg);
    }

    void publish_demodulation_frequency_debug()
    {
        std_msgs::msg::Float64 demodulation_frequency_msg;
        demodulation_frequency_msg.data = demodulation_frequency_hz_;
        demodulation_frequency_pub_->publish(demodulation_frequency_msg);
    }

    void update_homing_estimate(
        const double delta_range_m,
        const double delta_time_s,
        const rclcpp::Time & window_start_stamp,
        const rclcpp::Time & window_end_stamp)
    {
        Eigen::Vector3d start_position_m;
        Eigen::Vector3d end_position_m;
        {
            std::lock_guard<std::mutex> lock(dvl_mutex_);
            if (!interpolate_position(window_start_stamp, start_position_m) ||
                !interpolate_position(window_end_stamp, end_position_m))
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "homing update skipped: receive-time odometry/depth buffer does not cover audio window [%.6f, %.6f]",
                    window_start_stamp.seconds(),
                    window_end_stamp.seconds());
                return;
            }
        }

        const Eigen::Vector3d delta_position_m = end_position_m - start_position_m;
        const bool direction_observable =
            homing_direction_ekf_.update(delta_position_m, delta_range_m, delta_time_s);
        if (!direction_observable) {
            return;
        }

        const Eigen::Vector3d direction = homing_direction_ekf_.normalized_direction();
        if (direction.isZero()) {
            return;
        }
        const Eigen::Vector3d filtered_direction = filter_direction(direction);

        geometry_msgs::msg::Vector3Stamped direction_msg;
        direction_msg.header.stamp = this->now();
        direction_msg.header.frame_id = "dvl";
        direction_msg.vector.x = filtered_direction.x();
        direction_msg.vector.y = filtered_direction.y();
        direction_msg.vector.z = filtered_direction.z();
        homing_direction_pub_->publish(direction_msg);
    }

    Eigen::Vector3d filter_direction(const Eigen::Vector3d & direction)
    {
        const double alpha = std::clamp(direction_filter_alpha_, 0.0, 1.0);
        if (!have_filtered_direction_) {
            filtered_direction_ = direction;
            have_filtered_direction_ = true;
            return filtered_direction_;
        }

        filtered_direction_ = (1.0 - alpha) * filtered_direction_ + alpha * direction;
        const double norm = filtered_direction_.norm();
        if (norm < 1.0e-9) {
            filtered_direction_ = direction;
        } else {
            filtered_direction_ /= norm;
        }
        return filtered_direction_;
    }

    rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr homing_direction_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr demodulation_frequency_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr iq_snr_ratio_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr iq_coherence_pub_;

    std::vector<TimedSample> sample_buffer_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::thread worker_thread_;
    bool stop_worker_ = false;

    std::size_t channels_ = 2;
    std::size_t channel_index_ = 0;
    std::size_t bytes_per_sample_ = 4;
    std::size_t frame_size_ = channels_ * bytes_per_sample_;

    std::size_t window_size_ = 4096;
    std::size_t hop_size_ = 1024;
    std::uint64_t next_window_start_sample_ = 0;
    std::size_t sampling_rate_ = 96000;
    double reference_frequency_hz_ = 21164.0; //27211.0도 있을 수 있음.
    double sound_speed_mps_ = 1500.0; // 수조/해역에 맞춰 보정할 음속.
    double demodulation_frequency_hz_ = 21164.0;
    bool enable_frequency_acquisition_ = true;
    bool have_frequency_lock_ = false;
    double frequency_search_half_width_hz_ = 1000.0;
    double frequency_search_step_hz_ = 10.0;
    double frequency_reacquire_threshold_hz_ = 50.0;
    int frequency_lock_required_windows_ = 5;
    double frequency_lock_tolerance_hz_ = 20.0;
    double pending_frequency_hz_ = 0.0;
    int pending_frequency_count_ = 0;
    double min_iq_magnitude_ = 1.0e-8;
    double min_iq_snr_ratio_ = 2.0;
    double min_iq_coherence_ = 0.60;
    std::size_t coherence_segments_ = 8;
    double sync_delay_s_ = 0.10;
    double direction_filter_alpha_ = 0.12;

    bool have_previous_iq_ = false;
    std::complex<double> previous_iq_{0.0, 0.0};
    rclcpp::Time previous_iq_stamp_;
    HomingDirectionEkf homing_direction_ekf_;
    Eigen::Vector3d filtered_direction_{1.0, 0.0, 0.0};
    bool have_filtered_direction_ = false;
    std::mutex dvl_mutex_;
    std::deque<TimedVector2> odometry_buffer_;
    std::deque<TimedScalar> depth_buffer_;
    std::size_t max_pose_buffer_size_ = 200;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::AudioPhaseEstimatorNode)
