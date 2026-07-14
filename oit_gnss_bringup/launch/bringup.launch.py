"""ZED-F9P bringup; the u-blox driver is the sole serial-port owner."""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogError, LogInfo, OpaqueFunction
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).parent))
from profile_loader import driver_parameters, load_profile


_REQUIRED_NTRIP_ENV = (
    "ICHIMILL_HOST", "ICHIMILL_PORT", "ICHIMILL_MOUNTPOINT",
    "ICHIMILL_USER", "ICHIMILL_PASS",
)


def _bool(value: str, name: str) -> bool:
    if value.lower() not in {"true", "false"}:
        raise RuntimeError(f"{name} must be true or false")
    return value.lower() == "true"


def _use_ntrip(value: str, profile_enabled: bool) -> bool:
    if value == "auto":
        return profile_enabled
    return _bool(value, "use_ntrip")


def _setup(context):
    profile = load_profile(context.launch_configurations["gnss_profile"].strip())
    device = context.launch_configurations["gnss_device"].strip() or profile.device
    use_ntrip = _use_ntrip(context.launch_configurations["use_ntrip"].strip().lower(), profile.ntrip_enabled)
    actions = []
    if not profile.verified:
        actions.append(LogInfo(msg="WARNING: GNSS profile has not been verified on hardware"))
    actions.extend([
        Node(
            package="ublox_gps", executable="ublox_gps_node", namespace="/ublox",
            name="ublox_gps_node", parameters=[driver_parameters(profile, device)],
            # Confirmed against fixed ublox source: navpvt is private, NavSAT is navstate.
            remappings=[
                ("~/fix", "/fix_raw"),
                ("~/navpvt", "/ublox/navpvt"),
                ("navstate", "/ublox/navsat"),
                ("/rtcm", profile.rtcm_topic),
                ("rxmrawx", "/ublox/rxmrawx"),
            ], output="screen"),
        Node(package="oit_gnss_monitor", executable="fix_quality_monitor",
             parameters=[str(Path(__file__).parent.parent / "config" / "quality.yaml")], output="screen"),
    ])
    if not use_ntrip:
        return actions
    missing = [key for key in _REQUIRED_NTRIP_ENV if not os.environ.get(key)]
    if missing:
        message = "NTRIP client not started: required ICHIMILL environment variables are not set."
        if _bool(context.launch_configurations["strict_ntrip"].strip(), "strict_ntrip"):
            raise RuntimeError(message)
        actions.append(LogError(msg=message))
        return actions
    actions.append(Node(
        package="ntrip_client", executable="ntrip_ros.py", name="ntrip_client",
        parameters=[{
            "host": os.environ["ICHIMILL_HOST"], "port": int(os.environ["ICHIMILL_PORT"]),
            "mountpoint": os.environ["ICHIMILL_MOUNTPOINT"], "username": os.environ["ICHIMILL_USER"],
            "password": os.environ["ICHIMILL_PASS"], "authenticate": True,
            "rtcm_message_package": "rtcm_msgs", "reconnect_attempt_wait_seconds": profile.reconnect_delay_sec,
            "rtcm_timeout_seconds": profile.timeout_sec,
        }], remappings=[("rtcm", profile.rtcm_topic), ("fix", "/fix_raw")], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gnss_profile", default_value="f9p_ichimill"),
        DeclareLaunchArgument("gnss_device", default_value=""),
        DeclareLaunchArgument("use_ntrip", default_value="auto"),
        DeclareLaunchArgument("strict_ntrip", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
