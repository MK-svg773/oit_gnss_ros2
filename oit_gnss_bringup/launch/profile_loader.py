"""Validated, credential-free hardware profiles for GNSS bringup."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class GnssProfile:
    name: str
    verified: bool
    device: str
    frame_id: str
    baudrate: int
    measurement_rate_hz: float
    nav_rate: int
    dynamic_model: str
    nav_pvt: bool
    nav_sat: bool
    rxm_rawx: bool
    ntrip_enabled: bool
    rtcm_topic: str
    reconnect_delay_sec: float
    timeout_sec: float


_DYNAMIC_MODELS = {
    "portable", "stationary", "pedestrian", "automotive", "sea",
    "airborne1", "airborne2", "airborne4", "wristwatch",
}


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _nonempty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def _positive_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _positive_integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be boolean")
    return value


def validate_profile(data: Any, name: str = "profile") -> GnssProfile:
    root = _mapping(data, name)
    profile = _mapping(root.get("profile"), "profile")
    receiver = _mapping(root.get("receiver"), "receiver")
    publish = _mapping(root.get("publish"), "publish")
    ntrip = _mapping(root.get("ntrip"), "ntrip")
    dynamic_model = _nonempty_string(receiver.get("dynamic_model"), "receiver.dynamic_model")
    if dynamic_model not in _DYNAMIC_MODELS:
        raise ValueError("receiver.dynamic_model is not accepted by ublox_gps")
    return GnssProfile(
        name=name,
        verified=_boolean(profile.get("verified"), "profile.verified"),
        device=_nonempty_string(receiver.get("device"), "receiver.device"),
        frame_id=_nonempty_string(receiver.get("frame_id"), "receiver.frame_id"),
        baudrate=_positive_integer(receiver.get("baudrate"), "receiver.baudrate"),
        measurement_rate_hz=_positive_number(receiver.get("measurement_rate_hz"), "receiver.measurement_rate_hz"),
        nav_rate=_positive_integer(receiver.get("nav_rate"), "receiver.nav_rate"),
        dynamic_model=dynamic_model,
        nav_pvt=_boolean(publish.get("nav_pvt"), "publish.nav_pvt"),
        nav_sat=_boolean(publish.get("nav_sat"), "publish.nav_sat"),
        rxm_rawx=_boolean(publish.get("rxm_rawx"), "publish.rxm_rawx"),
        ntrip_enabled=_boolean(ntrip.get("enabled"), "ntrip.enabled"),
        rtcm_topic=_nonempty_string(ntrip.get("rtcm_topic"), "ntrip.rtcm_topic"),
        reconnect_delay_sec=_positive_number(ntrip.get("reconnect_delay_sec"), "ntrip.reconnect_delay_sec"),
        timeout_sec=_positive_number(ntrip.get("timeout_sec"), "ntrip.timeout_sec"),
    )


def driver_parameters(profile: GnssProfile, device: str) -> dict[str, object]:
    """Map profile units to the fixed KumarRobotics/ublox ROS 2 parameters."""
    return {
        "device": device,
        "frame_id": profile.frame_id,
        "uart1.baudrate": profile.baudrate,
        # ublox_gps declares rate in Hz and converts internally to milliseconds.
        "rate": float(profile.measurement_rate_hz),
        "nav_rate": profile.nav_rate,
        "dynamic_model": profile.dynamic_model,
        "publish.nav.pvt": profile.nav_pvt,
        "publish.nav.sat": profile.nav_sat,
        "publish.rxm.raw": profile.rxm_rawx,
        "publish.rxm.rtcm": True,
    }


def load_profile(name: str) -> GnssProfile:
    if Path(name).name != name or not name.replace("_", "").isalnum():
        raise ValueError("gnss_profile must contain only letters, digits, and underscores")
    path = Path(get_package_share_directory("oit_gnss_bringup")) / "config" / "profiles" / f"{name}.yaml"
    if not path.is_file():
        raise ValueError(f"gnss_profile '{name}' was not found")
    with path.open(encoding="utf-8") as stream:
        return validate_profile(yaml.safe_load(stream), name)
