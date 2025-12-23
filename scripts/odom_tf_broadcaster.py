#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # Parameters
        # Gazebo -> ROS odometry topic (your newcar)
        self.declare_parameter('odom_topic', '/model/newcar/odometry')

        # Optional overrides (leave empty to use msg.header.frame_id / msg.child_frame_id)
        self.declare_parameter('parent_frame_id_override', '')
        self.declare_parameter('child_frame_id_override', '')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_override = self.get_parameter('parent_frame_id_override').value
        self.child_override = self.get_parameter('child_frame_id_override').value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f"Publishing TF from Odometry topic: {self.odom_topic}")
        if self.parent_override or self.child_override:
            self.get_logger().info(
                f"Using overrides: parent='{self.parent_override}' child='{self.child_override}'"
            )
        else:
            self.get_logger().info("Using Odometry message frame_id / child_frame_id (recommended).")

    def odom_callback(self, msg: Odometry):
        # Choose frames
        parent = self.parent_override if self.parent_override else msg.header.frame_id
        child = self.child_override if self.child_override else msg.child_frame_id

        if not parent or not child:
            self.get_logger().warn(
                f"Cannot publish TF: empty frame(s). parent='{parent}' child='{child}'"
            )
            return

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
