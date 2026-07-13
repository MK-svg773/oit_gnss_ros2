# Architecture

`ublox_gps_node` is the only process that opens the receiver device.  The NTRIP client publishes `rtcm_msgs/msg/Message` on `/rtcm`; the u-blox driver subscribes and writes those bytes to that already-open serial connection.  No serial NTRIP bridge is used.

The driver fix is remapped to `/fix_raw`. `oit_gnss_monitor` validates latitude/longitude, status, covariance, freshness, monotonic stamp, and horizontal/vertical standard deviations, then publishes accepted samples only on `/fix` and emits diagnostics for rejections. Both fix topics use sensor-data QoS.

The GNSS bringup owns no TF, odometry, IMU, or lidar publishers. `oit_smartbot_diff_04_description` currently owns `base_link`; it contains no GPS link, so the antenna transform remains deliberately absent until measured and added to that URDF/Xacro.

External contract: `/fix`, `/ublox/navpvt`, `/ublox/navsat`, `/ublox/rxmrawx`. SmartBot owns `/odom`, `/imu`, `/tf`, `/tf_static`, and `/hesai/pandar`.
