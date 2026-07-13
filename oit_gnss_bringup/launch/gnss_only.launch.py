from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
def generate_launch_description(): return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("oit_gnss_bringup"),"launch","bringup.launch.py"])), launch_arguments={"gnss_profile":"f9p_standalone","use_ntrip":"false"}.items())])
