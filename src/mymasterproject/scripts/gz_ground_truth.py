#!/usr/bin/env python3
"""
gz_ground_truth.py

Subscribes directly to Gazebo transport /world/campus_world/pose/info,
finds ackermann_car by name, publishes true world pose to ROS.

Fix: gz-transport Node callbacks work automatically — no .spin() needed.
     Just keep the node alive while rclpy spins.
"""

import sys
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster

# Try gz-transport bindings (Gazebo Harmonic = transport13/msgs10)
GZ_AVAILABLE = False
GZ_VERSION = 0

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_AVAILABLE = True
    GZ_VERSION = 13
except ImportError:
    pass

if not GZ_AVAILABLE:
    try:
        from gz.transport12 import Node as GzNode
        from gz.msgs9.pose_v_pb2 import Pose_V
        GZ_AVAILABLE = True
        GZ_VERSION = 12
    except ImportError:
        pass

if not GZ_AVAILABLE:
    try:
        from gz.transport11 import Node as GzNode
        from gz.msgs8.pose_v_pb2 import Pose_V
        GZ_AVAILABLE = True
        GZ_VERSION = 11
    except ImportError:
        pass


class GzGroundTruth(Node):
    def __init__(self):
        super().__init__('gz_ground_truth')

        self.declare_parameter('gz_world_topic', '/world/campus_world/pose/info')
        self.declare_parameter('model_name',     'ackermann_car')
        self.declare_parameter('pose_topic',     '/ground_truth_pose')
        self.declare_parameter('path_topic',     '/ground_truth_path')
        self.declare_parameter('map_frame',      'map')
        self.declare_parameter('publish_tf',     True)
        self.declare_parameter('max_path',       500)

        self.gz_topic   = self.get_parameter('gz_world_topic').value
        self.model_name = self.get_parameter('model_name').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.map_frame  = self.get_parameter('map_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.max_path   = self.get_parameter('max_path').value

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        if self.publish_tf:
            self.tf_br = TransformBroadcaster(self)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame
        self.msg_count = 0
        self._gz_node = None

        self.get_logger().info('=' * 60)
        self.get_logger().info('GZ GROUND TRUTH NODE')
        self.get_logger().info(f'  gz topic:  {self.gz_topic}')
        self.get_logger().info(f'  model:     {self.model_name}')
        self.get_logger().info(f'  ROS pose:  {self.pose_topic}')
        self.get_logger().info(f'  publish_tf:{self.publish_tf}')
        self.get_logger().info('=' * 60)

        if GZ_AVAILABLE:
            self.get_logger().info(f'gz-transport v{GZ_VERSION} found — subscribing')
            self._start_gz_subscriber()
        else:
            self.get_logger().error(
                'gz-transport Python bindings NOT found!\n'
                'Install: sudo apt install python3-gz-transport13\n'
                'No ground truth will be published.'
            )

    def _start_gz_subscriber(self):
        """Subscribe to gz-transport topic. Callbacks fire automatically."""
        self._gz_node = GzNode()
        result = self._gz_node.subscribe(Pose_V, self.gz_topic, self._gz_pose_cb)
        if result:
            self.get_logger().info(f'Subscribed to gz topic: {self.gz_topic}')
        else:
            self.get_logger().error(f'Failed to subscribe to: {self.gz_topic}')

    def _gz_pose_cb(self, pose_v_msg):
        """Called from gz-transport thread when new pose arrives."""
        stamp = self.get_clock().now().to_msg()

        for pose in pose_v_msg.pose:
            if pose.name == self.model_name:
                self._publish(
                    stamp,
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
                break

    def _publish(self, stamp, x, y, z, qx, qy, qz, qw):
        self.msg_count += 1

        # PoseStamped
        ps = PoseStamped()
        ps.header.stamp       = stamp
        ps.header.frame_id    = self.map_frame
        ps.pose.position.x    = x
        ps.pose.position.y    = y
        ps.pose.position.z    = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        # Path
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > self.max_path:
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)

        # TF: map -> ground_truth
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp            = stamp
            t.header.frame_id         = self.map_frame
            t.child_frame_id          = 'ground_truth'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x    = qx
            t.transform.rotation.y    = qy
            t.transform.rotation.z    = qz
            t.transform.rotation.w    = qw
            self.tf_br.sendTransform(t)

        if self.msg_count == 1:
            yaw = math.atan2(
                2*(qw*qz + qx*qy),
                1 - 2*(qy*qy + qz*qz))
            self.get_logger().info(
                f'✓ First GT pose: ({x:.3f}, {y:.3f}, {z:.3f}) '
                f'yaw={math.degrees(yaw):.1f}°'
            )

        if self.msg_count % 200 == 0:
            self.get_logger().info(f'[{self.msg_count:6d}] GT: ({x:.3f}, {y:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = GzGroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()