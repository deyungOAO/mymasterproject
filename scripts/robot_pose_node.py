#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class RobotPoseNode(Node):
	def __init__(self):
		super().__init__('robot_pose_node')

		# Parameters
		self.declare_parameter('odom_topic', '/odom')
		self.declare_parameter('robot_pose_topic', '/robot_pose')

		odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
		robot_pose_topic = self.get_parameter('robot_pose_topic').get_parameter_value().string_value

		self.pose_pub = self.create_publisher(PoseStamped, robot_pose_topic, 10)
		self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

		self.get_logger().info(f'Listening to {odom_topic}, publishing PoseStamped to {robot_pose_topic}')

	def odom_callback(self, msg: Odometry):
		pose_msg = PoseStamped()
		pose_msg.header = msg.header
		pose_msg.pose = msg.pose.pose
		self.pose_pub.publish(pose_msg)


def main(args=None):
	rclpy.init(args=args)
	node = RobotPoseNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
