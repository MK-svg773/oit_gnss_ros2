# Hardware validation checklist

1. Confirm ZED-F9P enumerates, then use `/dev/serial/by-id/...`, not a transient `/dev/ttyACM0`.
2. Start `gnss_only.launch.py`; inspect NavPVT, NavSAT, raw measurements, and `/fix_raw` frame `gps`.
3. Set `ICHIMILL_HOST`, `ICHIMILL_PORT`, `ICHIMILL_MOUNTPOINT`, `ICHIMILL_USER`, and `ICHIMILL_PASS`; start bringup and confirm `/rtcm` plus receiver RTCM diagnostics.
4. Confirm RTK float/fixed from NavPVT/receiver diagnostics, then run `validate_topics` with the full SmartBot.
5. Record a short MCAP, replay it, and inspect types, TF, frames, and timestamps.

Unverified: receiver firmware/product, transport and baud rate, antenna extrinsics, caster mountpoint/VRS-GGA requirement, desired rate, and RTK fixed conditions.
