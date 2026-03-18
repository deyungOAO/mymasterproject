#!/usr/bin/env python3
# rename_id_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from copy import deepcopy

class RenameId(Node):
    def __init__(self):
        super().__init__('rename_id')
        # Input topic (PoseArray)
        self.input_topic = '/world/campus_world/dynamic_pose/info'
        # Output topic (PoseArray) — adjust if you prefer a different name
        self.output_topic = '/world/campus_world/dynamic_pose/info_world'

        self.sub = self.create_subscription(
            PoseArray, self.input_topic, self._callback, 10
        )
        self.pub = self.create_publisher(PoseArray, self.output_topic, 10)

        self.get_logger().info(
            f"Listening on {self.input_topic}; republishing to {self.output_topic} with frame_id='world'"
        )

    def _callback(self, msg: PoseArray):
        out = deepcopy(msg)
        out.header.frame_id = 'world'
        # Keep the original timestamp; only frame_id is changed
        self.pub.publish(out)

def main():
    rclpy.init()
    node = RenameId()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
