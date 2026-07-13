from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
TOPICS=["/fix","/fix_raw","/odom","/imu","/tf","/tf_static","/hesai/pandar","/ublox/navpvt","/ublox/navsat","/ublox/rxmrawx","/diagnostics","/ublox/navstatus","/ublox/rxmrtcm","/rtcm"]
def _record(context):
    arguments=["record","--output",LaunchConfiguration("output"),"--storage",LaunchConfiguration("storage")]
    compression=context.launch_configurations["compression"].strip()
    if compression: arguments += ["--compression-mode","file","--compression-format",compression]
    return [Node(package="rosbag2_transport",executable="ros2bag",arguments=arguments+TOPICS,output="screen")]
def generate_launch_description(): return LaunchDescription([DeclareLaunchArgument("output",default_value="rosbag2_gnss"),DeclareLaunchArgument("storage",default_value="mcap"),DeclareLaunchArgument("compression",default_value=""),OpaqueFunction(function=_record)])
