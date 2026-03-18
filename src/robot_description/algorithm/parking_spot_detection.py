#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import time
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool

class ParkingSpotDetection(Node):
    def __init__(self):
        super().__init__('parking_spot_detection')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera2/points',
            self.point_cloud_callback,
            1)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.spot_found_publisher = self.create_publisher(Bool, '/parking_spot_found', 10)
        
        self.moving_forward = True
        self.slowing_down = False
        self.super_slowing_down = False
        self.detecting_tail = False  # New state for tail detection
        
        self.forward_start_time = time.time()
        self.move_forward()

    def move_forward(self, speed: float = 0.8) -> None: 
        """Move the vehicle forward at the specified speed."""
        twist = Twist()
        twist.linear.x = speed  
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def point_cloud_callback(self, msg) -> None:
        """Processes incoming point cloud data."""
        if self.moving_forward:
            self.get_logger().info("Starting parking spot detection...")
            self.move_forward()

        # Convert PointCloud2 to a list of points
        points = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))
        
        if not self.detecting_tail:  
            if self.detect_open_space(points):
                self.detecting_tail = True  # Switch to tail detection mode
                self.get_logger().info("Open parking spot found!")
                self.get_logger().info("Now aligning with bumper.")
        else:
            if self.detect_tail_alignment(points):
                self.get_logger().info("Bumper alignment confirmed. Stopping vehicle.")
                self.stop_vehicle()

    def detect_open_space(self, points) -> bool:
        """Detects an open parking space from the point cloud data."""
        if not points:
            return False

        # Bounding box for open space detection
        x_min, x_max = 0.0, 2.6   
        y_min, y_max = -1.0, 1.3   
        z_min, z_max = 0, 1.5    

        # Filter points inside bounding box
        filtered_points = [p for p in points if (x_min <= p[0] <= x_max) and 
                                                    (y_min <= p[1] <= y_max) and 
                                                    (z_min <= p[2] <= z_max)]
        
        num_filtered = len(filtered_points)
        self.get_logger().info(f"Filtered points in box: {num_filtered}")

        # Thresholds
        stop_threshold = 1  
        slow_threshold = 15000  
        superslow_threshold = 10000 

        if num_filtered < stop_threshold:  
            return True  

        elif num_filtered < slow_threshold and not self.slowing_down:
            self.get_logger().info("Few points detected. Slowing down.")
            self.slow_down()

        elif num_filtered < superslow_threshold and not self.super_slowing_down:
            self.get_logger().info("Even fewer points detected. Slowing down more.")
            self.super_slow_down()

        elif num_filtered >= slow_threshold and (self.slowing_down or self.super_slowing_down):
            self.get_logger().info("Obstacle detected. Resuming normal speed.")
            self.resume_driving()

        return False

    def detect_tail_alignment(self, points) -> bool:
        """Detects the tail of a neighboring vehicle for proper alignment."""
        if not points:
            return False

        # **New bounding box for detecting tail alignment**
        x_min, x_max = 0.0, 1.0   
        y_min, y_max = -1.0, 0.0   
        z_min, z_max = 0.0, 1.5    

        # Filter points inside new bounding box
        filtered_points = [p for p in points if (x_min <= p[0] <= x_max) and 
                                                    (y_min <= p[1] <= y_max) and 
                                                    (z_min <= p[2] <= z_max)]

        num_filtered = len(filtered_points)
        self.get_logger().info(f"Filtered points in tail detection: {num_filtered}")

        tail_threshold = 500  

        return num_filtered > tail_threshold  

    def stop_vehicle(self) -> None:
        """Stops the vehicle and publishes a parking spot found signal."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Vehicle stopped.")

        self.spot_found_publisher.publish(Bool(data=True))
        self.get_logger().info("Published parking spot found signal.")
        self.destroy_node()

    def slow_down(self) -> None:
        """Slows down the vehicle to prepare for parking."""
        self.slowing_down = True
        self.move_forward(speed=0.2)  

    def super_slow_down(self) -> None:
        """Slows down the vehicle even more."""
        self.super_slowing_down = True
        self.move_forward(speed=0.05)  

    def resume_driving(self) -> None:
        """Resumes normal speed."""
        self.slowing_down = False
        self.super_slowing_down = False
        self.move_forward(speed=0.8)  

def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpotDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
