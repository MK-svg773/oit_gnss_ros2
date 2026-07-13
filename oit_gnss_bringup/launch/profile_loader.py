"""Validated GNSS profile loader; profiles contain no NTRIP credentials."""
from dataclasses import dataclass
import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory

@dataclass(frozen=True)
class GnssProfile:
    name: str; verified: bool; device: str; frame_id: str; baudrate: int
    measurement_rate_hz: float; nav_rate: int; dynamic_model: str
    nav_pvt: bool; nav_sat: bool; rxm_rawx: bool; ntrip_enabled: bool
    reconnect_delay_sec: float; timeout_sec: float

def _need(data, key, typ):
    value = data.get(key)
    if not isinstance(value, typ) or isinstance(value, bool): raise ValueError(f"{key} has an invalid type")
    return value

def validate_profile(data, name="profile"):
    if not isinstance(data, dict): raise ValueError("profile must be a mapping")
    profile, receiver, publish, ntrip = (data.get(k) for k in ("profile", "receiver", "publish", "ntrip"))
    if not all(isinstance(x, dict) for x in (profile, receiver, publish, ntrip)): raise ValueError("profile, receiver, publish, and ntrip are required mappings")
    device, frame_id = _need(receiver,"device",str).strip(), _need(receiver,"frame_id",str).strip()
    if not device or not frame_id: raise ValueError("receiver.device and receiver.frame_id must not be empty")
    rate = float(_need(receiver,"measurement_rate_hz",(int,float)))
    if not math.isfinite(rate) or rate <= 0: raise ValueError("receiver.measurement_rate_hz must be positive")
    baud, nav_rate = _need(receiver,"baudrate",int), _need(receiver,"nav_rate",int)
    if baud <= 0 or nav_rate <= 0: raise ValueError("receiver baudrate and nav_rate must be positive")
    if receiver.get("dynamic_model") not in {"portable","stationary","pedestrian","automotive","sea","airborne1","airborne2","airborne4","wrist","bike"}: raise ValueError("unsupported receiver.dynamic_model")
    for key in ("verified",):
        if not isinstance(profile.get(key), bool): raise ValueError(f"profile.{key} must be boolean")
    for key in ("nav_pvt","nav_sat","rxm_rawx"):
        if not isinstance(publish.get(key), bool): raise ValueError(f"publish.{key} must be boolean")
    for key in ("enabled",):
        if not isinstance(ntrip.get(key), bool): raise ValueError(f"ntrip.{key} must be boolean")
    reconnect, timeout = float(_need(ntrip,"reconnect_delay_sec",(int,float))), float(_need(ntrip,"timeout_sec",(int,float)))
    if reconnect <= 0 or timeout <= 0: raise ValueError("NTRIP timing values must be positive")
    return GnssProfile(name,profile["verified"],device,frame_id,baud,rate,nav_rate,receiver["dynamic_model"],publish["nav_pvt"],publish["nav_sat"],publish["rxm_rawx"],ntrip["enabled"],reconnect,timeout)

def load_profile(name):
    if Path(name).name != name or not name.replace("_", "").isalnum(): raise ValueError("gnss_profile must contain letters, digits, and underscores")
    path=Path(get_package_share_directory("oit_gnss_bringup"))/"config"/"profiles"/f"{name}.yaml"
    if not path.is_file(): raise ValueError(f"gnss_profile '{name}' was not found")
    with path.open(encoding="utf-8") as stream: return validate_profile(yaml.safe_load(stream), name)
