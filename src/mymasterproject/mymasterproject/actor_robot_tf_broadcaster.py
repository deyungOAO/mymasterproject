#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster


class ActorRobotTFBroadcaster(Node):
	def __init__(self):
		super().__init__('actor_robot_tf_broadcaster')

		# ========== Parameters ==========
		# Robot
		self.declare_parameter('robot_odom_topic', '/model/vehicle_blue/odometry')
		self.declare_parameter('robot_default_parent_frame', 'odom')      # if header.frame_id is empty
		self.declare_parameter('robot_default_child_frame', 'base_link')  # if child_frame_id is empty

		# Actors
		self.declare_parameter('actor_posearray_topic', '/actors/poses')
		self.declare_parameter('actor_parent_frame', '')          # '' = use PoseArray.header.frame_id
		self.declare_parameter('actor_frame_prefix', 'actor_')    # actor_0, actor_1, ...
		self.declare_parameter('actor_child_suffix', '')          # e.g. '/base_link' if you want actor_0/base_link

		self.robot_odom_topic = self.get_parameter('robot_odom_topic').get_parameter_value().string_value
		self.robot_default_parent = self.get_parameter('robot_default_parent_frame').get_parameter_value().string_value
		self.robot_default_child = self.get_parameter('robot_default_child_frame').get_parameter_value().string_value

		self.actor_topic = self.get_parameter('actor_posearray_topic').get_parameter_value().string_value
		self.actor_parent_frame = self.get_parameter('actor_parent_frame').get_parameter_value().string_value
		self.actor_prefix = self.get_parameter('actor_frame_prefix').get_parameter_value().string_value
		self.actor_child_suffix = self.get_parameter('actor_child_suffix').get_parameter_value().string_value

		# TF broadcaster
		self.tf_broadcaster = TransformBroadcaster(self)

		# Subscriptions
		self.robot_sub = self.create_subscription(
			Odometry,
			self.robot_odom_topic,
			self.robot_odom_cb,
			10
		)

		self.actor_sub = self.create_subscription(
			PoseArray,
			self.actor_topic,
			self.actor_cb,
			10
		)

		self.get_logger().info(f"Robot TF from [{self.robot_odom_topic}]")
		self.get_logger().info(f"Actors TF from PoseArray [{self.actor_topic}]")

	# ========== Robot TF: odom -> base_link ==========
	def robot_odom_cb(self, msg: Odometry):
		t = TransformStamped()
		t.header.stamp = msg.header.stamp

		# Use frames from Odometry if set, otherwise fallback
		parent = msg.header.frame_id if msg.header.frame_id else self.robot_default_parent
		child = msg.child_frame_id if msg.child_frame_id else self.robot_default_child

		t.header.frame_id = parent
		t.child_frame_id = child

		t.transform.translation.x = msg.pose.pose.position.x
		t.transform.translation.y = msg.pose.pose.position.y
		t.transform.translation.z = msg.pose.pose.position.z
		t.transform.rotation = msg.pose.pose.orientation

		self.tf_broadcaster.sendTransform(t)

	# ========== Actors TF: map -> actor_i ==========
	def actor_cb(self, msg: PoseArray):
		# parent frame: param > PoseArray.header.frame_id > default 'world'
		if self.actor_parent_frame:
			parent = self.actor_parent_frame
		elif msg.header.frame_id:
			parent = msg.header.frame_id
		else:
			parent = 'map'

		transforms = []

		for i, pose in enumerate(msg.poses):
			t = TransformStamped()
			t.header.stamp = msg.header.stamp
			t.header.frame_id = parent

			child_name = f"{self.actor_prefix}{i}"
			if self.actor_child_suffix:
				child_name = child_name + self.actor_child_suffix

			t.child_frame_id = child_name

			t.transform.translation.x = pose.position.x
			t.transform.translation.y = pose.position.y
			t.transform.translation.z = pose.position.z
			t.transform.rotation = pose.orientation  # use full orientation (if your actor has it)

			transforms.append(t)

		# Broadcast all actor transforms
		for t in transforms:
			self.tf_broadcaster.sendTransform(t)


def main(args=None):
	rclpy.init(args=args)
	node = ActorRobotTFBroadcaster()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
