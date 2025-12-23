#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class Peek(Node):
    def __init__(self):
        super().__init__('peek_dynamic_pose')
        self.sub = self.create_subscription(PoseArray,
            '/world/campus_world/dynamic_pose/info', self.cb, 10)

    def cb(self, msg: PoseArray):
        print(f'PoseArray len = {len(msg.poses)}')
        for i, p in enumerate(msg.poses[:30]):
            print(f'[{i:03d}] x={p.position.x:6.2f}  y={p.position.y:6.2f}  z={p.position.z:5.2f}')
        rclpy.shutdown()

def main():
    rclpy.init(); Peek(); rclpy.spin(rclpy.get_default_context())
if __name__ == '__main__':
    main()
