#!/usr/bin/env python3
"""Observe the SmartBot topic contract without publishing substitute data."""

import argparse
import math
import sys
import time
from dataclasses import dataclass, field

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2
from tf2_msgs.msg import TFMessage
from ublox_msgs.msg import NavPVT, NavSAT, RxmRAWX


@dataclass(frozen=True)
class Contract:
    message_type: str
    message_class: object
    qos: QoSProfile
    needs_frame: bool = False


@dataclass
class Observation:
    count: int = 0
    first_time: float | None = None
    last_time: float | None = None
    invalid_frame: bool = False
    fix: NavSatFix | None = None


TF_STATIC_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CONTRACTS = {
    "/fix": Contract("sensor_msgs/msg/NavSatFix", NavSatFix, qos_profile_sensor_data, True),
    "/odom": Contract("nav_msgs/msg/Odometry", Odometry, qos_profile_sensor_data, True),
    "/imu": Contract("sensor_msgs/msg/Imu", Imu, qos_profile_sensor_data, True),
    "/tf": Contract("tf2_msgs/msg/TFMessage", TFMessage, qos_profile_sensor_data),
    "/tf_static": Contract("tf2_msgs/msg/TFMessage", TFMessage, TF_STATIC_QOS),
    "/hesai/pandar": Contract("sensor_msgs/msg/PointCloud2", PointCloud2, qos_profile_sensor_data, True),
    "/ublox/navpvt": Contract("ublox_msgs/msg/NavPVT", NavPVT, qos_profile_sensor_data),
    "/ublox/navsat": Contract("ublox_msgs/msg/NavSAT", NavSAT, qos_profile_sensor_data),
    "/ublox/rxmrawx": Contract("ublox_msgs/msg/RxmRAWX", RxmRAWX, qos_profile_sensor_data),
}


class Validator(Node):
    def __init__(self) -> None:
        super().__init__("gnss_topic_validator")
        self.observations = {name: Observation() for name in CONTRACTS}
        self.subscriptions = []
        for name, contract in CONTRACTS.items():
            self.subscriptions.append(self.create_subscription(
                contract.message_class, name,
                lambda message, topic=name: self._observe(topic, message), contract.qos))

    def _observe(self, topic: str, message: object) -> None:
        observation = self.observations[topic]
        now = time.monotonic()
        observation.count += 1
        observation.first_time = observation.first_time or now
        observation.last_time = now
        if CONTRACTS[topic].needs_frame and not message.header.frame_id:
            observation.invalid_frame = True
        if topic == "/fix":
            observation.fix = message

    def validate(self, timeout_sec: float, require_fix: bool) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        discovered = dict(self.get_topic_names_and_types())
        success = True
        for topic, contract in CONTRACTS.items():
            types = discovered.get(topic, [])
            publishers = self.count_publishers(topic)
            observation = self.observations[topic]
            valid = contract.message_type in types and publishers > 0 and observation.count > 0
            rate = 0.0
            if observation.count > 1 and observation.first_time is not None and observation.last_time is not None:
                rate = (observation.count - 1) / (observation.last_time - observation.first_time)
            print(f"{'OK' if valid else 'MISSING'} {topic}: type={types}, publishers={publishers}, messages={observation.count}, rate={rate:.2f} Hz")
            if observation.invalid_frame:
                print(f"INVALID {topic}: header.frame_id is empty")
                valid = False
            success &= valid
        fix = self.observations["/fix"].fix
        if fix is None:
            print("MISSING /fix message")
            return False
        coordinates_valid = (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)
                             and -90.0 <= fix.latitude <= 90.0
                             and -180.0 <= fix.longitude <= 180.0)
        if not coordinates_valid:
            print("INVALID /fix: latitude or longitude is not finite/in range")
            success = False
        if fix.status.status == NavSatStatus.STATUS_NO_FIX:
            level = "FAIL" if require_fix else "WARN"
            print(f"{level} /fix: STATUS_NO_FIX")
            success &= not require_fix
        return bool(success)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--require-fix", action="store_true")
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    rclpy.init()
    node = Validator()
    try:
        valid = node.validate(args.timeout, args.require_fix)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if valid else 1)


if __name__ == "__main__":
    main()
