from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction


TOPICS = [
    "/fix", "/fix_raw", "/odom", "/imu", "/tf", "/tf_static", "/hesai/pandar",
    "/ublox/navpvt", "/ublox/navsat", "/ublox/rxmrawx", "/diagnostics",
    "/ublox/navstatus", "/ublox/rxmrtcm", "/rtcm",
]


def _record(context):
    command = ["ros2", "bag", "record", "--output", context.launch_configurations["output"],
               "--storage", context.launch_configurations["storage"]]
    mode = context.launch_configurations["compression_mode"].strip()
    fmt = context.launch_configurations["compression_format"].strip()
    if mode != "none":
        if mode not in {"file", "message"} or not fmt:
            raise RuntimeError("compression_mode must be none, file, or message; format is required when enabled")
        command.extend(["--compression-mode", mode, "--compression-format", fmt])
    return [ExecuteProcess(cmd=command + TOPICS, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value="rosbag2_gnss"),
        DeclareLaunchArgument("storage", default_value="mcap"),
        DeclareLaunchArgument("compression_mode", default_value="none"),
        DeclareLaunchArgument("compression_format", default_value=""),
        OpaqueFunction(function=_record),
    ])
