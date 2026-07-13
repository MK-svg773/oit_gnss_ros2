# Driver compatibility

Selected source revisions are KumarRobotics/ublox `3a3e1c253722b68e350a60aee6ef399d8c2304cb` (ROS 2 3.0.0) and LORD-MicroStrain/ntrip_client `ad5c1f0138792d8ba630d91129396063ea918191` (ROS 2 1.4.1). On Jazzy, `rosdep resolve rtcm_msgs` and `rosdep resolve nmea_msgs` resolve to `ros-jazzy-*` packages.

The u-blox source provides `publish.nav.pvt`, `publish.nav.sat`, and `publish.rxm.raw`; it subscribes to `/rtcm` as `rtcm_msgs/msg/Message`. It creates `navpvt` and `navsat` relative to namespace `/ublox`; the bringup remaps its private `~/fix` to `/fix_raw`.

Important limitation: this revision's F9P HPG product path does not register an `RxmRAWX` subscription. Its `RxmRAWX` publisher is only in `TimProduct`; `RawDataProduct` emits legacy `RxmRAW` and is only added for protocol <=14. ZED-F9P is newer. Renaming `rxmraw` cannot preserve the requested message type. A minimal upstream/fork patch must add `RxmRAWX` publisher/subscriber to the F9P firmware/product path before enabling the `/ublox/rxmrawx` runtime contract. This repository does not falsely claim hardware validation until that fork is pinned and built.

NTRIP client 1.4.1 supports Basic authentication, configurable NTRIP version, reconnect attempts/delay, RTCM timeout and SSL/TLS options. It derives/sends GGA from its `fix` subscription for VRS use. Bringup uses `/fix_raw` to avoid correction feedback through the quality gate. Credentials are read only from `ICHIMILL_*` environment variables and are never logged.
