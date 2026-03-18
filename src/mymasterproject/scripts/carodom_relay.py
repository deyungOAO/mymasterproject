#!/usr/bin/env python3
"""
carodom_relay.py

Republishes /carodomfromGazebo with frame_id changed to 'map'
so RViz displays it at the correct world position without
double-applying the map->odom offset.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class CarodomRelay(Node):
    def __init__(self):
        super().__init__('carodom_relay')

        self.declare_parameter('input_topic',  '/carodomfromGazebo')
        self.declare_parameter('output_topic', '/carodom_world')
        self.declare_parameter('frame_id',     'map')

        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self.cb, 10)

        self.get_logger().info(
            f'carodom_relay: {in_topic} -> {out_topic} '
            f'(frame_id overridden to "{self.frame_id}")'
        )

    def cb(self, msg: Odometry):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarodomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()