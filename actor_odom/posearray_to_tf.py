#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseArrayToTF(Node):
    def __init__(self):
        super().__init__('posearray_to_tf')
        self.parent = self.declare_parameter('parent_frame','map').get_parameter_value().string_value
        in_topic = self.declare_parameter('in_topic','/world/campus_world/dynamic_pose/info').get_parameter_value().string_value
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(PoseArray, in_topic, self.cb, 10)
        self.get_logger().info(f'Listening: {in_topic}, parent="{self.parent}"')

    def cb(self, pa: PoseArray):
        for i, p in enumerate(pa.poses):
            t = TransformStamped()
            t.header = pa.header   # uses /clock if you bridged it
            t.header.frame_id = self.parent
            t.child_frame_id = f"moving_{i}"
            t.transform.translation.x = p.position.x
            t.transform.translation.y = p.position.y
            t.transform.translation.z = p.position.z
            t.transform.rotation = p.orientation
            self.br.sendTransform(t)

def main():
    rclpy.init()
    node = PoseArrayToTF()
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
