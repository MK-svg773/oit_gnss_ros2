#!/usr/bin/env python3
"""Runtime contract checker. Reports missing SmartBot-owned topics; never publishes them."""
import math, sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
REQUIRED={"/fix":"sensor_msgs/msg/NavSatFix","/odom":"nav_msgs/msg/Odometry","/imu":"sensor_msgs/msg/Imu","/tf":"tf2_msgs/msg/TFMessage","/tf_static":"tf2_msgs/msg/TFMessage","/hesai/pandar":"sensor_msgs/msg/PointCloud2","/ublox/navpvt":"ublox_msgs/msg/NavPVT","/ublox/navsat":"ublox_msgs/msg/NavSAT","/ublox/rxmrawx":"ublox_msgs/msg/RxmRAWX"}
class Validator(Node):
 def __init__(self): super().__init__("gnss_topic_validator"); self.fix=None; self.create_subscription(NavSatFix,"/fix",lambda m:setattr(self,"fix",m),10)
 def run(self):
  time.sleep(2); found={n:ts for n,ts in self.get_topic_names_and_types()}; ok=True
  for name, typ in REQUIRED.items():
   actual=found.get(name,[]); good=typ in actual and self.count_publishers(name)>0
   print(f"{'OK' if good else 'MISSING'} {name}: expected {typ}, found {actual}, publishers={self.count_publishers(name)}")
   ok &= good
  end=time.monotonic()+3
  while self.fix is None and time.monotonic()<end: rclpy.spin_once(self,timeout_sec=.1)
  if self.fix is None: print("MISSING /fix message within 3 seconds"); ok=False
  elif not (math.isfinite(self.fix.latitude) and math.isfinite(self.fix.longitude) and self.fix.header.frame_id): print("INVALID /fix: non-finite coordinate or empty frame_id"); ok=False
  return ok
def main():
 rclpy.init(); n=Validator(); success=n.run(); n.destroy_node(); rclpy.shutdown(); sys.exit(0 if success else 1)
if __name__=="__main__": main()
