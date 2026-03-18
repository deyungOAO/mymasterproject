#!/usr/bin/env python3
"""
robot_pose_node.py - FIXED VERSION for Static TF Setup

Converts odometry + TF to robot pose in map frame for path planning.
Works with static TF (map -> odom).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import math


class RobotPoseNode(Node):
    def __init__(self):
        super().__init__('robot_pose_node')
        
        # ============================================
        # PARAMETERS
        # ============================================
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ackermann_car/base_footprint')
        
        odom_topic = self.get_parameter('odom_topic').value
        robot_pose_topic = self.get_parameter('robot_pose_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # ============================================
        # TF BUFFER AND LISTENER
        # ============================================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ============================================
        # SUBSCRIBERS
        # ============================================
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
        
        # ============================================
        # PUBLISHERS
        # ============================================
        self.pose_publisher = self.create_publisher(
            PoseStamped, 
            robot_pose_topic, 
            10)
        
        # ============================================
        # STATE
        # ============================================
        self.last_pose = None
        self.message_count = 0
        self.last_log_time = self.get_clock().now()
        self.tf_working = False
        
        # ============================================
        # STARTUP
        # ============================================
        self.get_logger().info('=' * 60)
        self.get_logger().info('ROBOT POSE NODE STARTED (STATIC TF MODE)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Subscribing to: {odom_topic}')
        self.get_logger().info(f'Publishing pose to: {robot_pose_topic}')
        self.get_logger().info(f'Map frame: {self.map_frame}')
        self.get_logger().info(f'Base frame: {self.base_frame}')
        self.get_logger().info('=' * 60)
    
    def odom_callback(self, msg):
        """Process odometry and transform to map frame using TF."""
        
        self.message_count += 1
        
        try:
            # Look up transform from map to base_footprint
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),  # Get latest available
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            if not self.tf_working:
                self.tf_working = True
                self.get_logger().info('✓ TF transform available!')
            
            # ============================================
            # CREATE POSE IN MAP FRAME
            # ============================================
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.map_frame
            
            # Position from TF
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            
            # Orientation from TF
            pose_msg.pose.orientation = transform.transform.rotation
            
            # Store for monitoring
            self.last_pose = pose_msg
            
            # Publish robot pose
            self.pose_publisher.publish(pose_msg)
            
            # ============================================
            # LOGGING
            # ============================================
            
            # Log first message
            if self.message_count == 1:
                self.get_logger().info('=' * 60)
                self.get_logger().info('✓ FIRST POSE PUBLISHED')
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'Frame: {pose_msg.header.frame_id}')
                self.get_logger().info(f'Position: x={pose_msg.pose.position.x:.3f}, '
                                     f'y={pose_msg.pose.position.y:.3f}, '
                                     f'z={pose_msg.pose.position.z:.3f}')
                self.get_logger().info('=' * 60)
            
            # Periodic logging (every 2 seconds)
            current_time = self.get_clock().now()
            time_diff = (current_time - self.last_log_time).nanoseconds / 1e9
            
            if time_diff >= 2.0:
                # Calculate yaw from quaternion
                quat = pose_msg.pose.orientation
                yaw = math.atan2(
                    2.0 * (quat.w * quat.z + quat.x * quat.y),
                    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
                )
                
                self.get_logger().info(
                    f'[{self.message_count:6d}] Pose in {self.map_frame}: '
                    f'({pose_msg.pose.position.x:7.2f}, '
                    f'{pose_msg.pose.position.y:7.2f}, {math.degrees(yaw):6.1f}°)'
                )
                
                self.last_log_time = current_time
                
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if self.message_count % 50 == 1:  # Only log occasionally to avoid spam
                self.get_logger().warn(
                    f'⚠ TF lookup failed: {str(e)[:50]}... '
                    f'Waiting for TF: {self.map_frame} -> {self.base_frame}'
                )
    
    def get_current_pose(self):
        """Return the most recent pose (useful for other nodes)."""
        return self.last_pose


def main(args=None):
    """Main function."""
    
    rclpy.init(args=args)
    node = RobotPoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('=' * 60)
        node.get_logger().info(f'Shutting down... (processed {node.message_count} poses)')
        node.get_logger().info('=' * 60)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()