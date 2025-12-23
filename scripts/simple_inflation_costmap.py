#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid


class SimpleInflationCostmap(Node):
    def __init__(self):
        super().__init__('simple_inflation_costmap')

        # Parameters
        self.declare_parameter('inflation_radius', 5)   # meters
        self.declare_parameter('robot_radius', 1.4)       # meters
        self.declare_parameter('cost_scaling_factor', 2.0)
        self.declare_parameter('input_map_topic', '/map')
        self.declare_parameter('output_costmap_topic', '/global_costmap/costmap')

        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.cost_scaling_factor = float(self.get_parameter('cost_scaling_factor').value)
        self.input_map_topic = self.get_parameter('input_map_topic').value
        self.output_costmap_topic = self.get_parameter('output_costmap_topic').value

        self.get_logger().info(
            f"Inflation node started | inflation={self.inflation_radius} m "
            f"robot_radius={self.robot_radius} m scaling={self.cost_scaling_factor}"
        )

        # ---------- QoS ----------
        qos_latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribe to map (latched)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.input_map_topic,
            self.map_callback,
            qos_latched
        )

        # Publish costmap (latched)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            self.output_costmap_topic,
            qos_latched
        )

        self.map_received = False

    def map_callback(self, msg: OccupancyGrid):
        if self.map_received:
            return

        self.map_received = True
        self.get_logger().info("Received map → computing inflated costmap")

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        data = np.array(msg.data, dtype=np.int16).reshape((height, width))

        # Obstacle mask
        occ_thresh = 65
        obstacles = (data >= occ_thresh).astype(np.uint8)

        # Distance transform (meters)
        free = (obstacles == 0).astype(np.uint8)
        dist_pixels = cv2.distanceTransform(free, cv2.DIST_L2, 5)
        dist_meters = dist_pixels * resolution

        # Cost computation
        cost = np.zeros_like(dist_meters, dtype=np.float32)
        cost[obstacles == 1] = 100.0

        mask = (obstacles == 0) & (dist_meters <= self.inflation_radius)
        dist_rel = np.maximum(dist_meters[mask] - self.robot_radius, 0.0)
        cost_vals = 100.0 * np.exp(-self.cost_scaling_factor * dist_rel)
        cost[mask] = np.clip(cost_vals, 0.0, 100.0)

        cost_int = cost.astype(np.int8)

        # Publish
        costmap = OccupancyGrid()
        costmap.header = msg.header
        costmap.info = msg.info
        costmap.data = cost_int.flatten().tolist()

        self.costmap_pub.publish(costmap)
        self.get_logger().info(
            f"Published inflated costmap on {self.output_costmap_topic} "
            f"({width}x{height})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleInflationCostmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
