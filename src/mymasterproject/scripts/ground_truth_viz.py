#!/usr/bin/env python3
"""
ground_truth_viz.py

Publishes Gazebo ground truth pose in the MAP frame for RViz visualization.

Since /gz_odom gives pose in ackermann_car/odom frame, and
map->odom is a fixed offset (spawn_x, spawn_y), we simply add
the offset to get the true map-frame position.

Publishes:
  /ground_truth_pose   geometry_msgs/PoseStamped   (for RViz Pose display)
  /ground_truth_path   nav_msgs/Path                (trail of past poses)

Add in RViz:
  - Pose display  → topic /ground_truth_pose  (shows arrow)
  - Path display  → topic /ground_truth_path  (shows trail)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class GroundTruthViz(Node):
    def __init__(self):
        super().__init__('ground_truth_viz')

        self.declare_parameter('gz_odom_topic',        '/gz_odom')
        self.declare_parameter('pose_topic',           '/ground_truth_pose')
        self.declare_parameter('path_topic',           '/ground_truth_path')
        self.declare_parameter('map_frame',            'map')
        self.declare_parameter('spawn_x',              -20.0)
        self.declare_parameter('spawn_y',                0.0)
        self.declare_parameter('spawn_yaw',              0.0)
        self.declare_parameter('max_path_length',      500)   # trail length
        self.declare_parameter('publish_tf',           True)  # also publish TF

        self.gz_odom_topic  = self.get_parameter('gz_odom_topic').value
        self.pose_topic     = self.get_parameter('pose_topic').value
        self.path_topic     = self.get_parameter('path_topic').value
        self.map_frame      = self.get_parameter('map_frame').value
        self.spawn_x        = self.get_parameter('spawn_x').value
        self.spawn_y        = self.get_parameter('spawn_y').value
        self.spawn_yaw      = self.get_parameter('spawn_yaw').value
        self.max_path       = self.get_parameter('max_path_length').value
        self.publish_tf     = self.get_parameter('publish_tf').value

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        # Optional TF broadcaster for ground truth frame
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.sub = self.create_subscription(
            Odometry, self.gz_odom_topic, self.odom_cb, 10)

        # Path trail
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        self.msg_count = 0
        self.create_timer(5.0, self.watchdog)

        self.get_logger().info('=' * 60)
        self.get_logger().info('GROUND TRUTH VISUALIZER')
        self.get_logger().info(f'  odom input:   {self.gz_odom_topic}')
        self.get_logger().info(f'  pose output:  {self.pose_topic}')
        self.get_logger().info(f'  path output:  {self.path_topic}')
        self.get_logger().info(
            f'  spawn offset: ({self.spawn_x}, {self.spawn_y}, '
            f'{math.degrees(self.spawn_yaw):.1f}°)'
        )
        self.get_logger().info('In RViz add:')
        self.get_logger().info(f'  Pose display → {self.pose_topic}')
        self.get_logger().info(f'  Path display → {self.path_topic}')
        self.get_logger().info('=' * 60)

    def watchdog(self):
        if self.msg_count == 0:
            self.get_logger().warn(
                f'No messages on {self.gz_odom_topic}. '
                f'Check bridge_gz_odom is running.'
            )

    @staticmethod
    def quat_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    @staticmethod
    def yaw_to_quat(yaw, q):
        """Fill quaternion message in-place from yaw."""
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)

    def odom_cb(self, msg: Odometry):
        self.msg_count += 1

        # Odom-frame pose
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        oz = msg.pose.pose.position.z
        odom_yaw = self.quat_to_yaw(msg.pose.pose.orientation)

        # Transform to map frame by applying spawn offset
        # map_pos = R_spawn * odom_pos + spawn_translation
        cy = math.cos(self.spawn_yaw)
        sy = math.sin(self.spawn_yaw)
        mx = cy * ox - sy * oy + self.spawn_x
        my = sy * ox + cy * oy + self.spawn_y
        mz = oz
        map_yaw = odom_yaw + self.spawn_yaw

        stamp = msg.header.stamp

        # ── PoseStamped ──────────────────────────────────────────────────
        ps = PoseStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = mx
        ps.pose.position.y = my
        ps.pose.position.z = mz
        self.yaw_to_quat(map_yaw, ps.pose.orientation)
        self.pose_pub.publish(ps)

        # ── Path trail ───────────────────────────────────────────────────
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > self.max_path:
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)

        # ── TF: ground_truth frame (optional, for direct comparison) ─────
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp            = stamp
            t.header.frame_id         = self.map_frame
            t.child_frame_id          = 'ground_truth'
            t.transform.translation.x = mx
            t.transform.translation.y = my
            t.transform.translation.z = mz
            self.yaw_to_quat(map_yaw, t.transform.rotation)
            self.tf_broadcaster.sendTransform(t)

        if self.msg_count == 1:
            self.get_logger().info(
                f'✓ First ground truth pose: '
                f'map=({mx:.3f}, {my:.3f}) '
                f'yaw={math.degrees(map_yaw):.1f}°'
            )

        if self.msg_count % 200 == 0:
            self.get_logger().info(
                f'[{self.msg_count:6d}] '
                f'GT map: ({mx:.3f}, {my:.3f}, '
                f'{math.degrees(map_yaw):.1f}°) | '
                f'odom: ({ox:.3f}, {oy:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()