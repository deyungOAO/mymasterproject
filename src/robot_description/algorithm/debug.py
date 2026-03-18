#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class ParkingSpotDetection(Node):
    def __init__(self):
        super().__init__('parking_spot_detection')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera2/points',
            self.point_cloud_callback,
            20)
        self.subscription

    def point_cloud_callback(self, msg) -> None:
        """Processes incoming point cloud data without moving the vehicle."""
        self.get_logger().info("Processing point cloud data...")

        # Convert PointCloud2 to a list of points
        points = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))

        # Run bounding box check
        self.detect_open_space(points)

    def detect_open_space(self, points) -> None:
        """Adjust and analyze the bounding box of the depth camera."""
        if not points:
            self.get_logger().info("No points received.")
            return

        #Parameters for Lineup_car
        # Define bounding box limits
        x_min, x_max = 0.0, 2.6   # Forward distance to check (calibrated to match vehicle width)
        # **(-) is backwards of vehicle, (+) is forwards of vehicle**
        y_min, y_max = -1.0, 0  # Width range (calibrated to match vehicle length) 
        z_min, z_max = 0.0, 1.5   # Height range

        # #Parameters for Parking_Spot_Detection
        # # Define bounding box limits
        # x_min, x_max = 0.0, 2.6   # Forward distance to check (calibrated to match vehicle width)
        # # **(-) is backwards of vehicle, (+) is forwards of vehicle**
        # y_min, y_max = -1.0, 0.95  # Width range (calibrated to match vehicle length) 
        # z_min, z_max = 0.0, 1.5   # Height range

        #original bounding box limits (no factor of safety)
        # x_min, x_max = 0.0, 2.6   # Forward distance to check (calibrated to match vehicle width)
        # # **(-) is backwards of vehicle, (+) is forwards of vehicle**
        # y_min, y_max = -1.0, 0.95  # Width range (calibrated to match vehicle length) 
        # z_min, z_max = 0.0, 1.5   # Height range

        # Filter points inside bounding box
        filtered_points = [p for p in points if (x_min <= p[0] <= x_max) and 
                                                    (y_min <= p[1] <= y_max) and 
                                                    (z_min <= p[2] <= z_max)]

        num_filtered = len(filtered_points)
        self.get_logger().info(f"Filtered points in bounding box: {num_filtered}")

def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpotDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
