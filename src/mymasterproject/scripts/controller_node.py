#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


class SimpleControllerNode(Node):
    def __init__(self):
        super().__init__('simple_controller_node')

        # Parameters
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 2.0)

        robot_pose_topic = self.get_parameter(
            'robot_pose_topic').get_parameter_value().string_value
        path_topic = self.get_parameter(
            'path_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value

        self.lookahead_distance = self.get_parameter(
            'lookahead_distance').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter(
            'max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter(
            'max_angular_speed').get_parameter_value().double_value
        self.k_linear = self.get_parameter(
            'k_linear').get_parameter_value().double_value
        self.k_angular = self.get_parameter(
            'k_angular').get_parameter_value().double_value

        # Subscribers / publisher
        self.pose_sub = self.create_subscription(
            PoseStamped, robot_pose_topic, self.pose_callback, 10)
        self.path_sub = self.create_subscription(
            Path, path_topic, self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # State
        self.current_pose = None
        self.current_path = None
        self.current_path_idx = 0

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'SimpleControllerNode started. Pose: {robot_pose_topic}, '
            f'Path: {path_topic}, Cmd vel: {cmd_vel_topic}'
        )

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def path_callback(self, msg: Path):
        self.current_path = msg
        self.current_path_idx = 0
        self.get_logger().info(
            f'New path received with {len(msg.poses)} poses')

    def control_loop(self):
        if self.current_pose is None or self.current_path is None:
            return

        if len(self.current_path.poses) == 0:
            return

        target_pose, reached_goal = self.get_lookahead_target()

        if reached_goal:
            # stop at goal
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        # Robot pose
        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        robot_yaw = self.get_yaw(self.current_pose.pose)

        # Target pose
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - robot_yaw)

        # Simple P-controller on distance and heading
        linear = self.k_linear * distance
        angular = self.k_angular * yaw_error

        # Limit speeds
        linear = max(-self.max_linear_speed,
                     min(self.max_linear_speed, linear))
        angular = max(-self.max_angular_speed,
                      min(self.max_angular_speed, angular))

        # Very close to final point: stop linear
        if distance < 0.05:
            linear = 0.0

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def get_lookahead_target(self):
        """
        Return (target_pose, reached_goal_bool).
        """
        poses = self.current_path.poses
        if self.current_path_idx >= len(poses):
            return poses[-1], True

        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y

        # Find the first path point that is at least lookahead_distance away
        for i in range(self.current_path_idx, len(poses)):
            x = poses[i].pose.position.x
            y = poses[i].pose.position.y
            d = math.sqrt((x - robot_x) ** 2 + (y - robot_y) ** 2)
            if d >= self.lookahead_distance:
                self.current_path_idx = i
                return poses[i], False

        # If none are that far, use the final pose
        self.current_path_idx = len(poses) - 1
        return poses[-1], False

    @staticmethod
    def get_yaw(pose):
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = SimpleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
