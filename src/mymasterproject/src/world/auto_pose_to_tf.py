#!/usr/bin/env python3
import re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster

WORLD = 'campus_world'
PATTERN = re.compile(rf'^/world/{WORLD}/model/(actor_walking\d+)/pose$')

class AutoPoseToTF(Node):
    def __init__(self):
        super().__init__('auto_pose_to_tf')
        self.parent = self.declare_parameter('parent_frame','map').get_parameter_value().string_value
        self.br = TransformBroadcaster(self)
        self.subs = {}
        self.timer = self.create_timer(1.0, self.scan_topics)

    def scan_topics(self):
        for topic, types in self.get_topic_names_and_types():
            if 'geometry_msgs/msg/Pose' in types:
                m = PATTERN.match(topic)
                if m and topic not in self.subs:
                    name = m.group(1)
                    self.subs[topic] = self.create_subscription(
                        Pose, topic, lambda msg, n=name: self.pose_cb(msg, n), 10
                    )
                    self.get_logger().info(f'Subscribed: {topic} -> child={name}')

    def pose_cb(self, pose: Pose, name: str):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = name
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = AutoPoseToTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
