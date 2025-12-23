#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class PoseToOdom(Node):
	def __init__(self):
		super().__init__('pose_to_odom')

		self.declare_parameter('pose_topic', '/vehicle_blue_pose')
		self.declare_parameter('odom_topic', '/model/vehicle_blue/odometry')
		self.declare_parameter('odom_frame', 'odom')
		self.declare_parameter('base_frame', 'base_link')

		pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
		self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
		self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
		self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

		self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
		self.tf_broadcaster = TransformBroadcaster(self)

		self.subscription = self.create_subscription(
			PoseStamped,
			pose_topic,
			self.pose_callback,
			10
		)

	def pose_callback(self, msg: PoseStamped):
		# Build Odometry message
		odom = Odometry()
		odom.header.stamp = msg.header.stamp
		odom.header.frame_id = self.odom_frame
		odom.child_frame_id = self.base_frame

		odom.pose.pose = msg.pose  # copy position + orientation

		# (Velocities left at 0; Nav2 is ok with that)

		self.odom_pub.publish(odom)

		# Also broadcast TF: odom -> base_link
		t = TransformStamped()
		t.header.stamp = msg.header.stamp
		t.header.frame_id = self.odom_frame
		t.child_frame_id = self.base_frame
		t.transform.translation.x = msg.pose.position.x
		t.transform.translation.y = msg.pose.position.y
		t.transform.translation.z = msg.pose.position.z
		t.transform.rotation = msg.pose.orientation

		self.tf_broadcaster.sendTransform(t)


def main(args=None):
	rclpy.init(args=args)
	node = PoseToOdom()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
