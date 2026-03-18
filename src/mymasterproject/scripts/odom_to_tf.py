#!/usr/bin/env python3
"""
odom_to_tf.py - SIMPLE VERSION

Only does one thing: republishes odometry as odom->base_footprint TF.
map->odom is handled by static_tf_map_odom in the launch file.

No world pose correction, no StaticTransformBroadcaster, no fallback timer.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('gz_odom_topic', '/odom')

        odom_topic = self.get_parameter('gz_odom_topic').value

        self.tf = TransformBroadcaster(self)
        self.count = 0

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)

        self.get_logger().info(f'odom_to_tf: listening on {odom_topic}')

    def odom_cb(self, msg: Odometry):
        self.count += 1

        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = msg.header.frame_id.strip()
        t.child_frame_id  = msg.child_frame_id.strip()
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation
        self.tf.sendTransform(t)

        if self.count == 1:
            self.get_logger().info(
                f'✓ Publishing TF: {t.header.frame_id} -> {t.child_frame_id}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()