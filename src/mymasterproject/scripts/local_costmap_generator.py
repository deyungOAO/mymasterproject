#!/usr/bin/env python3
"""
local_costmap_generator.py - Creates a local costmap around the robot

This node:
1. Takes laser scan data
2. Creates a small costmap around robot (e.g., 10x10m)
3. Marks obstacles from laser
4. Inflates obstacles for safety
5. Publishes as OccupancyGrid for path planning

Use this with your PRM/DWA planners for local obstacle avoidance.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose
import numpy as np
import math


class LocalCostmapGenerator(Node):
    def __init__(self):
        super().__init__('local_costmap_generator')
        
        # ==================== PARAMETERS ====================
        
        # Costmap size
        self.declare_parameter('costmap_width', 20.0)    # meters
        self.declare_parameter('costmap_height', 20.0)   # meters
        self.declare_parameter('resolution', 0.1)        # meters per cell
        
        # Obstacle parameters
        self.declare_parameter('robot_radius', 1.2)      # Robot radius (m)
        self.declare_parameter('inflation_radius', 1.5)  # Inflation around obstacles (m)
        self.declare_parameter('obstacle_threshold', 0.8) # Min range to consider obstacle (m)
        
        # Frame IDs
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'ackermann_car/base_footprint')
        
        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/carodom_world')
        self.declare_parameter('costmap_topic', '/local_costmap')
        
        self.declare_parameter('update_frequency', 5.0)  # Hz
        
        # Get parameters
        self.costmap_width = self.get_parameter('costmap_width').value
        self.costmap_height = self.get_parameter('costmap_height').value
        self.resolution = self.get_parameter('resolution').value
        
        self.robot_radius = self.get_parameter('robot_radius').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        
        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        costmap_topic = self.get_parameter('costmap_topic').value
        
        update_freq = self.get_parameter('update_frequency').value
        
        # ==================== COSTMAP SETUP ====================
        
        # Calculate grid dimensions
        self.width_cells = int(self.costmap_width / self.resolution)
        self.height_cells = int(self.costmap_height / self.resolution)
        
        # Costmap grid (starts empty)
        self.costmap = np.zeros((self.height_cells, self.width_cells), dtype=np.int8)
        
        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Latest scan
        self.latest_scan = None
        
        # ==================== SUBSCRIBERS ====================
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        # ==================== PUBLISHERS ====================
        
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            costmap_topic,
            10
        )
        
        # ==================== TIMER ====================
        
        self.timer = self.create_timer(
            1.0 / update_freq,
            self.update_costmap
        )
        
        # ==================== LOGGING ====================
        
        self.update_count = 0
        self.last_log_time = self.get_clock().now()
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('LOCAL COSTMAP GENERATOR STARTED')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Costmap size: {self.costmap_width}m x {self.costmap_height}m')
        self.get_logger().info(f'Resolution: {self.resolution}m/cell ({self.width_cells}x{self.height_cells} cells)')
        self.get_logger().info(f'Robot radius: {self.robot_radius}m')
        self.get_logger().info(f'Inflation radius: {self.inflation_radius}m')
        self.get_logger().info(f'Update frequency: {update_freq} Hz')
        self.get_logger().info(f'Publishing to: {costmap_topic}')
        self.get_logger().info('=' * 70)
    
    def quaternion_to_yaw(self, q):
        """Extract yaw from quaternion."""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
    
    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
    
    def scan_callback(self, msg):
        """Store latest laser scan."""
        self.latest_scan = msg
    
    def world_to_grid(self, x, y):
        """
        Convert world coordinates to grid coordinates.
        Grid is centered on robot.
        """
        # Robot is at center of costmap
        center_x = self.width_cells // 2
        center_y = self.height_cells // 2
        
        # Offset from robot
        dx = x - self.robot_x
        dy = y - self.robot_y
        
        # Convert to grid coordinates
        grid_x = int(center_x + dx / self.resolution)
        grid_y = int(center_y + dy / self.resolution)
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates."""
        center_x = self.width_cells // 2
        center_y = self.height_cells // 2
        
        # Offset from center
        dx = (grid_x - center_x) * self.resolution
        dy = (grid_y - center_y) * self.resolution
        
        # World coordinates
        x = self.robot_x + dx
        y = self.robot_y + dy
        
        return x, y
    
    def is_valid_cell(self, grid_x, grid_y):
        """Check if grid coordinates are valid."""
        return (0 <= grid_x < self.width_cells and 
                0 <= grid_y < self.height_cells)
    
    def add_obstacle_to_costmap(self, x, y):
        """Add obstacle point to costmap."""
        grid_x, grid_y = self.world_to_grid(x, y)
        
        if self.is_valid_cell(grid_x, grid_y):
            self.costmap[grid_y, grid_x] = 100  # Occupied
    
    def inflate_costmap(self):
        """
        Inflate obstacles for safety.
        
        Cost decreases with distance from obstacle:
        - 100 = lethal (obstacle + robot_radius)
        - 99-1 = inflation zone
        - 0 = free
        """
        inflated = np.copy(self.costmap)
        
        # Inflation radius in cells
        inflation_cells = int(self.inflation_radius / self.resolution)
        robot_radius_cells = int(self.robot_radius / self.resolution)
        
        # Find all occupied cells
        occupied_cells = np.argwhere(self.costmap == 100)
        
        for oy, ox in occupied_cells:
            # Inflate around this obstacle
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    nx = ox + dx
                    ny = oy + dy
                    
                    if not self.is_valid_cell(nx, ny):
                        continue
                    
                    # Distance to obstacle
                    dist = math.sqrt(dx*dx + dy*dy) * self.resolution
                    
                    if dist <= self.robot_radius:
                        # Lethal zone
                        inflated[ny, nx] = 100
                    elif dist <= self.inflation_radius:
                        # Inflation zone - cost decreases with distance
                        # Cost = 99 at robot_radius, 1 at inflation_radius
                        ratio = (self.inflation_radius - dist) / (self.inflation_radius - self.robot_radius)
                        cost = int(1 + ratio * 98)
                        inflated[ny, nx] = max(inflated[ny, nx], cost)
        
        return inflated
    
    def update_costmap(self):
        """Update costmap from latest scan data."""
        
        if self.latest_scan is None:
            return
        
        self.update_count += 1
        
        # Clear costmap
        self.costmap.fill(0)
        
        # Process laser scan
        scan = self.latest_scan
        
        for i, r in enumerate(scan.ranges):
            # Skip invalid ranges
            if math.isnan(r) or math.isinf(r):
                continue
            
            # Skip too close (might be robot itself) or too far
            if r < self.obstacle_threshold or r > scan.range_max:
                continue
            
            # Calculate obstacle position in robot frame
            angle = scan.angle_min + i * scan.angle_increment
            
            # Transform to world frame
            obstacle_x_robot = r * math.cos(angle)
            obstacle_y_robot = r * math.sin(angle)
            
            # Rotate to world frame
            cos_yaw = math.cos(self.robot_yaw)
            sin_yaw = math.sin(self.robot_yaw)
            
            obstacle_x = self.robot_x + (obstacle_x_robot * cos_yaw - obstacle_y_robot * sin_yaw)
            obstacle_y = self.robot_y + (obstacle_x_robot * sin_yaw + obstacle_y_robot * cos_yaw)
            
            # Add to costmap
            self.add_obstacle_to_costmap(obstacle_x, obstacle_y)
        
        # Inflate obstacles
        self.costmap = self.inflate_costmap()
        
        # Publish costmap
        self.publish_costmap()
        
        # Logging
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_log_time).nanoseconds / 1e9
        
        if time_diff >= 2.0:
            occupied_cells = np.sum(self.costmap == 100)
            inflated_cells = np.sum((self.costmap > 0) & (self.costmap < 100))
            
            self.get_logger().info(
                f'[{self.update_count:5d}] '
                f'Robot: ({self.robot_x:7.2f}, {self.robot_y:7.2f}, {math.degrees(self.robot_yaw):6.1f}°) | '
                f'Obstacles: {occupied_cells} cells | '
                f'Inflated: {inflated_cells} cells'
            )
            self.last_log_time = current_time
    
    def publish_costmap(self):
        """Publish costmap as OccupancyGrid."""
        
        msg = OccupancyGrid()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        
        # Metadata
        msg.info.resolution = self.resolution
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        
        # Origin is bottom-left of costmap
        # Since costmap is centered on robot, origin is offset
        msg.info.origin.position.x = self.robot_x - self.costmap_width/2 
        msg.info.origin.position.y = self.robot_y - self.costmap_height/2
        msg.info.origin.position.z = 0.0
        
        msg.info.origin.orientation.w = 1.0
        
        # Data (flatten row-major)
        msg.data = self.costmap.flatten().tolist()
        
        self.costmap_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmapGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down local costmap generator...')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()