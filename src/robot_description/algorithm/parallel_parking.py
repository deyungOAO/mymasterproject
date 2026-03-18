#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # Import Bool message type
import time

class ParallelParking(Node):
    def __init__(self):
        super().__init__('parallel_parking')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.parking_spot_subscriber = self.create_subscription(
            Bool,
            '/parking_spot_found',
            self.parking_spot_callback,
            10)
        self.get_logger().info("Waiting for parking spot signal...")
    
    def parking_spot_callback(self, msg):
        if msg.data:
            self.get_logger().info("Executing parking maneuver...")
            self.execute_movements()
    
    def execute_movements(self):
        movements = [
            # ("Backing up and turning right", -0.3, 0.5, 3),  # ("Action name", linear velocity, angular velocity, duration)
            # ("Turning left", 0.2, 0.5, 2),
            # ("Moving backward", -0.5, 0.0, 3),
            # ("Turning right", 0.2, -0.5, 2),
            # ("Backing up and turning left", -0.3, 0.5, 2),
            # ("Backing up and turning right", -0.3, -0.5, 2),
            # ("Stopping", 0.0, 0.0, 0)
        ]
        movements.append(("Stopping", 0.0, 0.0, 1))      
        movements.append(("Backing up and turning right", -0.2, 0.2, 3))
        for _ in range(5):
            movements.append(("Backing up and turning right", -0.3, 0.5, 1))
        for _ in range(2):
            movements.append(("Backing up and turning right", -0.3, 0.2, 1))
        movements.append(("Stopping", 0.0, 0.0, 0))
        for _ in range(2):
            movements.append(("Moving backward", -0.5, 0.0, 1))
        movements.append(("Stopping", 0.0, 0.0, 0))
        movements.append(("Backing up and turning left", -0.15, -0.3, 2))
        for _ in range(12):
            movements.append(("Backing up and turning left", -0.15, -0.4, 1))
        movements.append(("Backing up and turning left", -0.3, -0.3, 3))
        movements.append(("Stopping", 0.0, 0.0, 0))
        movements.append(("Moving Forward", 0.3, 0.0, 2))

        for action, linear, angular, duration in movements:
            self.get_logger().info(action)
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher.publish(twist)
            time.sleep(duration)
        
        self.get_logger().info("Parallel parking sequence complete. Stopping node.")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ParallelParking()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
