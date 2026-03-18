#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DrivingDirections(Node):
    def __init__(self):
        super().__init__('driving_directions')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.execute_movements()
    
    def execute_movements(self):
        movements = [
            # ("Moving forward", 0.5, 0.0, 3), # ("Action name", linear velocity, angular velocity, duration)
            # ("Turning left", 0.2, 0.5, 2),
            ("Moving backward", -0.5, 0.0, 3),
            # ("Turning right", 0.2, -0.5, 2),
            ("Backing up and turning left", -0.3, 0.2, 3),
            # ("Backing up and turning right", -0.3, -0.5, 2),
            ("Stopping", 0.0, 0.0, 0)
        ]
        
        for action, linear, angular, duration in movements:
            self.get_logger().info(action)
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher.publish(twist)
            time.sleep(duration)
        
        self.get_logger().info("Sequence complete. Stopping the node.")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DrivingDirections()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
