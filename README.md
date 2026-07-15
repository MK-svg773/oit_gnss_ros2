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

Dependency revisions are fixed in `dependencies.repos`. The pinned u-blox source must receive the reviewed F9P HPG RxmRAWX patch before build:

```bash
src/oit_gnss_ros2/scripts/apply_ublox_f9p_patch.sh
```

## Start, validate, and record

Use `/dev/serial/by-id/...`, not a transient `/dev/ttyACM0`.

Start the robot/teleop first, then start GNSS in another terminal. This ensures that
the robot topics as well as the GNSS topics are available before recording starts.

```bash
# GNSS only
ros2 launch oit_gnss_bringup gnss_only.launch.py \
  gnss_device:=/dev/serial/by-id/usb-...

# Or GNSS with Ichimill NTRIP corrections
export ICHIMILL_HOST=example.invalid ICHIMILL_PORT=2101 ICHIMILL_MOUNTPOINT=mountpoint
export ICHIMILL_USER=user ICHIMILL_PASS='password'
ros2 launch oit_gnss_bringup bringup.launch.py \
  gnss_device:=/dev/serial/by-id/usb-... use_ntrip:=true

# Confirm the GNSS and required robot topics
ros2 run oit_gnss_bringup validate_topics
```

### Record rosbag

After teleop and GNSS have been started, record every public ROS topic with:

```bash
ros2 launch oit_rosbag record_bag.launch.py
```

The bag is stored as MCAP under `~/bags` by default. This command records all
public topics, including `/fix`, `/fix_raw`, `/ublox/navpvt`, `/ublox/navsat`,
`/ublox/rxmrawx`, odometry, IMU, TF, LiDAR, and camera topics while they are
published. Use `Ctrl+C` in the recording terminal to stop; wait for rosbag2 to
finish writing metadata before closing the terminal.

Set `OIT_ROSBAG_ROOT` before starting the command to store bags on a different
disk, for example `export OIT_ROSBAG_ROOT=/mnt/ssd/bags`.

Credentials are environment-only and are never logged. Missing variables disable
only NTRIP; use `use_ntrip:=false` for GNSS alone.

`validate_topics` reports missing SmartBot-owned topics rather than publishing fake data. Follow [hardware validation](docs/hardware_validation.md) for device, GNSS, NTRIP, RTK, TF, and rosbag checks. Exact firmware, transport/baud, antenna extrinsics, caster/GGA requirement, desired rate, and RTK-fixed conditions remain unverified. Add a measured `base_link -> gps` transform in `oit_smartbot_diff_04_description`; this package emits none.
