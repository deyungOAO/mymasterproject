#!/usr/bin/env python3
import re
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from gz_msgs.msg import Pose_V  # from ros-gz-interfaces

class ActorTFBroadcaster(Node):
    def __init__(self):
        super().__init__('actor_tf_broadcaster')
        self.parent = self.declare_parameter('parent_frame', 'map').get_parameter_value().string_value
        # include actors and your vehicle; edit as you like
        self.regex  = re.compile(self.declare_parameter('name_regex', r'^(actor_|vehicle_blue$)').get_parameter_value().string_value)
        in_topic    = self.declare_parameter('in_topic','/world/campus_world/dynamic_pose/info').get_parameter_value().string_value

        self.br  = TransformBroadcaster(self)
        self.sub = self.create_subscription(Pose_V, in_topic, self.cb, 10)
        self.get_logger().info(f'Listening: {in_topic}, parent="{self.parent}", filter="{self.regex.pattern}"')

    def cb(self, msg: Pose_V):
        now = self.get_clock().now().to_msg()
        for p in msg.pose:
            if not p.name or not self.regex.search(p.name):
                continue
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.parent
            t.child_frame_id = p.name.replace('::','/').replace(' ','_')
            t.transform.translation.x = p.position.x
            t.transform.translation.y = p.position.y
            t.transform.translation.z = p.position.z
            t.transform.rotation.x = p.orientation.x
            t.transform.rotation.y = p.orientation.y
            t.transform.rotation.z = p.orientation.z
            t.transform.rotation.w = p.orientation.w
            self.br.sendTransform(t)

def main():
    rclpy.init()
    node = ActorTFBroadcaster()
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
