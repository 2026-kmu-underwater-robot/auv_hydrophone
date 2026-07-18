#!/usr/bin/env python3
"""Publish bag-derived hydrophone noise with a position-dependent pinger tone."""

from __future__ import annotations

import math
from array import array
from pathlib import Path

import numpy as np
import rclpy
from audio_common_msgs.msg import AudioData, AudioDataStamped, AudioInfo
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions


INT32_SCALE = 2147483648.0
INT32_MIN = np.iinfo(np.int32).min
INT32_MAX = np.iinfo(np.int32).max


def distance_attenuated_amplitude(
    source_amplitude: float,
    distance_m: float,
    minimum_distance_m: float,
) -> float:
    """Return spherical-spreading amplitude with a near-source cap."""
    return source_amplitude / max(distance_m, minimum_distance_m)


def mix_pinger_tone(
    pcm: bytes,
    channels: int,
    sample_rate_hz: int,
    frequency_hz: float,
    source_amplitude: float,
    distance_m: float,
    minimum_distance_m: float,
    sound_speed_mps: float,
    first_sample_index: int,
) -> bytes:
    """Mix a propagation-delayed tone into interleaved S32LE PCM."""
    samples = np.frombuffer(pcm, dtype="<i4")
    valid_sample_count = (samples.size // channels) * channels
    if valid_sample_count == 0:
        return pcm

    frames_i32 = samples[:valid_sample_count].reshape(-1, channels)
    frames = frames_i32.astype(np.float64) / INT32_SCALE
    frame_indices = first_sample_index + np.arange(frames.shape[0], dtype=np.float64)
    emission_time_s = frame_indices / float(sample_rate_hz)
    propagation_delay_s = distance_m / sound_speed_mps
    amplitude = distance_attenuated_amplitude(
        source_amplitude, distance_m, minimum_distance_m
    )
    tone = amplitude * np.sin(
        2.0 * math.pi * frequency_hz * (emission_time_s - propagation_delay_s)
    )
    mixed = np.clip(frames + tone[:, np.newaxis], -1.0, 1.0)
    mixed_i32 = np.clip(
        np.rint(mixed * INT32_SCALE), INT32_MIN, INT32_MAX
    ).astype("<i4")
    trailing = pcm[valid_sample_count * np.dtype("<i4").itemsize :]
    return mixed_i32.reshape(-1).tobytes() + trailing


class NoiseBagReader:
    """Stream one audio topic from a rosbag and reopen it at EOF."""

    def __init__(self, bag_path: str, audio_topic: str) -> None:
        self.bag_path = str(Path(bag_path).expanduser().resolve())
        self.audio_topic = audio_topic
        if not Path(self.bag_path).is_dir():
            raise FileNotFoundError(f"noise bag directory not found: {self.bag_path}")
        self.reader: SequentialReader | None = None
        self._open()

    def _open(self) -> None:
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=self.bag_path, storage_id="sqlite3"),
            ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        topic_types = {
            topic.name: topic.type for topic in reader.get_all_topics_and_types()
        }
        expected_type = "audio_common_msgs/msg/AudioData"
        actual_type = topic_types.get(self.audio_topic)
        if actual_type != expected_type:
            raise RuntimeError(
                f"{self.audio_topic!r} has type {actual_type!r}; "
                f"expected {expected_type!r}"
            )
        self.reader = reader

    def next_audio_bytes(self) -> bytes:
        for pass_index in range(2):
            if self.reader is None:
                self._open()
            while self.reader is not None and self.reader.has_next():
                topic, serialized, _ = self.reader.read_next()
                if topic == self.audio_topic:
                    message = deserialize_message(serialized, AudioData)
                    return bytes(message.data)
            self.reader = None
            if pass_index == 0:
                self._open()
        raise RuntimeError(f"no messages found on {self.audio_topic!r}")


class PingerBuoyAudioSim(Node):
    """Replay empirical noise and add a fixed-world-position acoustic source."""

    def __init__(self) -> None:
        super().__init__("pinger_buoy_audio_sim")

        bag_path = self.declare_parameter(
            "noise_bag",
            "/home/kim/new_hydrophone_ws/localization_20260707_193328",
        ).value
        noise_topic = self.declare_parameter("noise_topic", "/audio").value
        audio_topic = self.declare_parameter("audio_topic", "/audio").value
        stamped_topic = self.declare_parameter(
            "audio_stamped_topic", "/audio_stamped"
        ).value
        audio_info_topic = self.declare_parameter(
            "audio_info_topic", "/audio_info"
        ).value
        odometry_topic = self.declare_parameter(
            "odometry_topic", "/odometry/filtered"
        ).value

        self.sample_rate_hz = int(
            self.declare_parameter("sample_rate_hz", 96000).value
        )
        self.channels = int(self.declare_parameter("channels", 2).value)
        self.frames_per_message = int(
            self.declare_parameter("frames_per_message", 960).value
        )
        self.frequency_hz = float(
            self.declare_parameter("frequency_hz", 21134.0).value
        )
        self.sound_speed_mps = float(
            self.declare_parameter("sound_speed_mps", 1500.0).value
        )
        self.source_amplitude = float(
            self.declare_parameter("source_amplitude", 0.03).value
        )
        self.minimum_distance_m = float(
            self.declare_parameter("minimum_distance_m", 0.5).value
        )
        self.pinger_position = np.array(
            [
                float(self.declare_parameter("pinger_x", 2.5).value),
                float(self.declare_parameter("pinger_y", 1.4).value),
                float(self.declare_parameter("pinger_z", -0.5).value),
            ],
            dtype=np.float64,
        )

        if self.sample_rate_hz <= 0 or self.channels <= 0:
            raise ValueError("sample_rate_hz and channels must be positive")
        if self.frames_per_message <= 0:
            raise ValueError("frames_per_message must be positive")
        if self.frequency_hz <= 0.0 or self.frequency_hz >= 0.5 * self.sample_rate_hz:
            raise ValueError("frequency_hz must be between zero and Nyquist")
        if self.sound_speed_mps <= 0.0 or self.minimum_distance_m <= 0.0:
            raise ValueError("sound_speed_mps and minimum_distance_m must be positive")
        if self.source_amplitude < 0.0:
            raise ValueError("source_amplitude must be non-negative")

        self.noise_reader = NoiseBagReader(str(bag_path), str(noise_topic))
        self.auv_position: np.ndarray | None = None
        self.first_sample_index = 0
        self.expected_pcm_bytes = self.frames_per_message * self.channels * 4

        self.audio_pub = self.create_publisher(AudioData, str(audio_topic), 10)
        self.stamped_pub = self.create_publisher(
            AudioDataStamped, str(stamped_topic), 10
        )
        info_qos = QoSProfile(depth=1)
        info_qos.reliability = ReliabilityPolicy.RELIABLE
        info_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.info_pub = self.create_publisher(
            AudioInfo, str(audio_info_topic), info_qos
        )
        self.odometry_sub = self.create_subscription(
            Odometry, str(odometry_topic), self.odometry_callback, 20
        )

        self.publish_audio_info()
        period_s = self.frames_per_message / float(self.sample_rate_hz)
        self.timer = self.create_timer(period_s, self.publish_audio)
        self.get_logger().info(
            f"Pinger buoy audio sim ready: {self.frequency_hz:.1f} Hz at "
            f"({self.pinger_position[0]:.2f}, {self.pinger_position[1]:.2f}, "
            f"{self.pinger_position[2]:.2f}), noise={self.noise_reader.bag_path}, "
            f"chunk={self.frames_per_message} frames."
        )

    def odometry_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.auv_position = np.array(
            [position.x, position.y, position.z], dtype=np.float64
        )

    def publish_audio_info(self) -> None:
        message = AudioInfo()
        message.channels = self.channels
        message.sample_rate = self.sample_rate_hz
        message.sample_format = "S32LE"
        message.bitrate = 128
        message.coding_format = "wave"
        self.info_pub.publish(message)

    def publish_audio(self) -> None:
        try:
            pcm = self.noise_reader.next_audio_bytes()
            if len(pcm) != self.expected_pcm_bytes:
                raise RuntimeError(
                    f"noise chunk has {len(pcm)} bytes; "
                    f"expected {self.expected_pcm_bytes}"
                )

            if self.auv_position is not None:
                distance_m = float(
                    np.linalg.norm(self.auv_position - self.pinger_position)
                )
                pcm = mix_pinger_tone(
                    pcm=pcm,
                    channels=self.channels,
                    sample_rate_hz=self.sample_rate_hz,
                    frequency_hz=self.frequency_hz,
                    source_amplitude=self.source_amplitude,
                    distance_m=distance_m,
                    minimum_distance_m=self.minimum_distance_m,
                    sound_speed_mps=self.sound_speed_mps,
                    first_sample_index=self.first_sample_index,
                )

            payload = array("B", pcm)
            plain_message = AudioData()
            plain_message.data = payload
            self.audio_pub.publish(plain_message)

            stamped_message = AudioDataStamped()
            stamped_message.header.stamp = self.get_clock().now().to_msg()
            stamped_message.header.frame_id = "hydrophone"
            stamped_message.audio.data = payload
            self.stamped_pub.publish(stamped_message)
            self.first_sample_index += self.frames_per_message
        except Exception as error:  # noqa: BLE001 - stop on malformed simulation input.
            self.get_logger().fatal(f"Failed to publish simulated audio: {error}")
            self.timer.cancel()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: PingerBuoyAudioSim | None = None
    try:
        node = PingerBuoyAudioSim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
