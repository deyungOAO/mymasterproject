#!/usr/bin/env python3
"""
ground_truth_to_tf.py

Subscribes to /gz_world_poses (tf2_msgs/TFMessage bridged from
/world/campus_world/dynamic_pose/info) and republishes the
ackermann_car entry as:
    map -> ackermann_car/base_footprint

Zero odometry integration error. Simulation only.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class GroundTruthToTF(Node):
    def __init__(self):
        super().__init__('ground_truth_to_tf')

        self.declare_parameter('gz_poses_topic', '/gz_world_poses')
        self.declare_parameter('model_name',     'ackermann_car')
        self.declare_parameter('parent_frame',   'map')
        self.declare_parameter('child_frame',    'ackermann_car/base_footprint')

        self.gz_topic     = self.get_parameter('gz_poses_topic').value
        self.model_name   = self.get_parameter('model_name').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame  = self.get_parameter('child_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.msg_count  = 0
        self.found_count = 0

        self.sub = self.create_subscription(
            TFMessage, self.gz_topic, self.cb, 10)

        # watchdog
        self.create_timer(5.0, self.watchdog)

        self.get_logger().info('=' * 60)
        self.get_logger().info('GROUND TRUTH -> TF')
        self.get_logger().info(f'  listening: {self.gz_topic}')
        self.get_logger().info(f'  model:     {self.model_name}')
        self.get_logger().info(f'  TF output: {self.parent_frame} -> {self.child_frame}')
        self.get_logger().info('=' * 60)

    def watchdog(self):
        if self.msg_count == 0:
            self.get_logger().warn(
                f'No messages on {self.gz_topic}. '
                f'Check bridge is running and world name is correct.'
            )
        elif self.found_count == 0:
            self.get_logger().warn(
                f'Receiving poses but "{self.model_name}" not found. '
                f'Check model_name parameter.'
            )

    def cb(self, msg: TFMessage):
        self.msg_count += 1

        for transform in msg.transforms:
            # dynamic_pose/info uses model name as child_frame_id
            if self.model_name in transform.child_frame_id:
                self.found_count += 1

                t = TransformStamped()
                t.header.stamp    = transform.header.stamp
                t.header.frame_id = self.parent_frame
                t.child_frame_id  = self.child_frame

                t.transform.translation = transform.transform.translation
                t.transform.rotation    = transform.transform.rotation

                self.tf_broadcaster.sendTransform(t)

                if self.found_count == 1:
                    self.get_logger().info(
                        f'✓ Ground truth active. '
                        f'First pos: ({t.transform.translation.x:.3f}, '
                        f'{t.transform.translation.y:.3f})'
                    )
                break


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()