import os, sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogError, OpaqueFunction
from launch_ros.actions import Node
sys.path.insert(0, str(Path(__file__).parent)); from profile_loader import load_profile

def setup(context):
    profile=load_profile(context.launch_configurations["gnss_profile"])
    device=context.launch_configurations["gnss_device"].strip() or profile.device
    use_ntrip=context.launch_configurations["use_ntrip"].strip().lower() == "true"
    params={"device":device,"frame_id":profile.frame_id,"uart1.baudrate":profile.baudrate,"rate":int(1000/profile.measurement_rate_hz),"nav_rate":profile.nav_rate,"dynamic_model":profile.dynamic_model,"publish.nav.pvt":profile.nav_pvt,"publish.nav.sat":profile.nav_sat,"publish.rxm.raw":profile.rxm_rawx,"publish.rxm.rtcm":True}
    actions=[Node(package="ublox_gps", executable="ublox_gps_node", namespace="/ublox", name="ublox_gps_node", parameters=[params], remappings=[("~/fix","/fix_raw"),("rxmraw","/ublox/rxmrawx")], output="screen"), Node(package="oit_gnss_monitor", executable="fix_quality_monitor", parameters=[Path(__file__).parent.parent/"config"/"quality.yaml"], output="screen")]
    if use_ntrip:
        missing=[key for key in ("ICHIMILL_HOST","ICHIMILL_PORT","ICHIMILL_MOUNTPOINT","ICHIMILL_USER","ICHIMILL_PASS") if not os.environ.get(key)]
        if missing: actions.append(LogError(msg="NTRIP disabled: required ICHIMILL environment variables are not set (credential values are never logged)."))
        else: actions.append(Node(package="ntrip_client", executable="ntrip_ros.py", name="ntrip_client", parameters=[{"host":os.environ["ICHIMILL_HOST"],"port":int(os.environ["ICHIMILL_PORT"]),"mountpoint":os.environ["ICHIMILL_MOUNTPOINT"],"username":os.environ["ICHIMILL_USER"],"password":os.environ["ICHIMILL_PASS"],"authenticate":True,"rtcm_message_package":"rtcm_msgs","reconnect_attempt_wait_seconds":profile.reconnect_delay_sec,"rtcm_timeout_seconds":profile.timeout_sec}], remappings=[("rtcm","/rtcm"),("fix","/fix_raw")], output="screen"))
    return actions
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("gnss_profile",default_value="f9p_ichimill"),DeclareLaunchArgument("gnss_device",default_value=""),DeclareLaunchArgument("use_ntrip",default_value="true"),OpaqueFunction(function=setup)])
