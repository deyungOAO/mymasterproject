#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
NAMES = ["vehicle_blue","actor_walking1","actor_walking2","actor_walking3","actor_walking4"]

def yaw_from_quat(q):
    # Z-yaw from quaternion
    s = 2.0*(q.w*q.z + q.x*q.y)
    c = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(s, c)

class Reader(Node):
    def __init__(self):
        super().__init__("print_tracked_poses")
        for n in NAMES:
            self.create_subscription(PoseStamped, f"/tracked/{n}/pose",
                                     lambda msg, name=n: self.cb(msg, name), 10)
    def cb(self, msg, name):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = yaw_from_quat(q)
        self.get_logger().info(f"{name:15s}  x={p.x:6.2f} y={p.y:6.2f} z={p.z:4.2f}  yaw={yaw:6.2f} rad")

def main():
    rclpy.init(); rclpy.spin(Reader()); rclpy.shutdown()
if __name__ == "__main__":
    main()
