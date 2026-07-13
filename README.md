# oit_gnss_ros2

ROS 2 Jazzy bringup for a u-blox ZED-F9P and Ichimill NTRIP. It owns only `/fix`, `/fix_raw`, `/ublox/navpvt`, `/ublox/navsat`, `/ublox/rxmrawx`, and diagnostics—never odometry, IMU, lidar, or TF.

## Install

```bash
cd ~/ros2_ws
vcs import src < src/oit_gnss_ros2/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to oit_gnss_bringup oit_gnss_monitor
source install/setup.bash
```

Dependency revisions are fixed in `dependencies.repos`. Read [driver compatibility](docs/driver_compatibility.md) before hardware use: the selected upstream u-blox revision needs an F9P RAWX patch to meet that final topic contract.

## Start and operate

Use `/dev/serial/by-id/...`, not a transient `/dev/ttyACM0`.

```bash
ros2 launch oit_gnss_bringup gnss_only.launch.py
ros2 launch oit_gnss_bringup bringup.launch.py gnss_device:=/dev/serial/by-id/usb-...
export ICHIMILL_HOST=example.invalid ICHIMILL_PORT=2101 ICHIMILL_MOUNTPOINT=mountpoint
export ICHIMILL_USER=user ICHIMILL_PASS='password'
ros2 launch oit_gnss_bringup bringup.launch.py use_ntrip:=true
ros2 run oit_gnss_bringup validate_topics
ros2 launch oit_gnss_bringup record.launch.py output:=rosbag2_gnss storage:=mcap
```

Credentials are environment-only and are never logged. Missing variables disable only NTRIP; use `use_ntrip:=false` for GNSS alone. Record explicitly with:

```bash
ros2 bag record /fix /fix_raw /odom /imu /tf /tf_static /hesai/pandar /ublox/navpvt /ublox/navsat /ublox/rxmrawx /diagnostics
```

`validate_topics` reports missing SmartBot-owned topics rather than publishing fake data. Follow [hardware validation](docs/hardware_validation.md) for device, GNSS, NTRIP, RTK, TF, and rosbag checks. Exact firmware, transport/baud, antenna extrinsics, caster/GGA requirement, desired rate, and RTK-fixed conditions remain unverified. Add a measured `base_link -> gps` transform in `oit_smartbot_diff_04_description`; this package emits none.
