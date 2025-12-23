#!/usr/bin/env python3
import math
import heapq
import random
import numpy as np
from typing import List, Tuple, Optional, Set
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class PRMPlannerNode(Node):
    def __init__(self):
        super().__init__('prm_planner_node')

        # Parameters
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')

        # Obstacle detection parameters
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('occupied_threshold', 50)
        
        # PRM parameters
        self.declare_parameter('num_samples', 500)  # Number of random samples
        self.declare_parameter('connection_radius_m', 2.0)  # Max distance to connect nodes (meters)
        self.declare_parameter('max_neighbors', 10)  # Max neighbors per node
        self.declare_parameter('rebuild_on_costmap', True)  # Rebuild roadmap when costmap changes
        
        # Robot safety parameters
        self.declare_parameter('inflation_radius_m', 1.4)  # Robot size
        self.declare_parameter('safety_distance_m', 0.3)  # Minimum clearance
        self.declare_parameter('collision_check_resolution', 0.1)  # Resolution for edge collision checking (meters)
        
        # Path quality
        self.declare_parameter('smooth_path', True)
        self.declare_parameter('cost_weight', 5.0)

        # Get parameters
        costmap_topic = self.get_parameter('costmap_topic').value
        map_topic = self.get_parameter('map_topic').value
        robot_pose_topic = self.get_parameter('robot_pose_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        path_topic = self.get_parameter('path_topic').value

        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.num_samples = int(self.get_parameter('num_samples').value)
        self.connection_radius_m = float(self.get_parameter('connection_radius_m').value)
        self.max_neighbors = int(self.get_parameter('max_neighbors').value)
        self.rebuild_on_costmap = bool(self.get_parameter('rebuild_on_costmap').value)
        self.inflation_radius_m = float(self.get_parameter('inflation_radius_m').value)
        self.safety_distance_m = float(self.get_parameter('safety_distance_m').value)
        self.collision_check_resolution = float(self.get_parameter('collision_check_resolution').value)
        self.smooth_path = bool(self.get_parameter('smooth_path').value)
        self.cost_weight = float(self.get_parameter('cost_weight').value)

        # Cell-based parameters (calculated from resolution)
        self.inflation_radius_cells = None
        self.safety_distance_cells = None
        self.connection_radius_cells = None
        self.collision_check_cells = None

        # QoS for latched OccupancyGrid
        grid_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscribers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, costmap_topic, self.costmap_callback, grid_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, grid_qos)
        self.robot_pose_sub = self.create_subscription(
            PoseStamped, robot_pose_topic, self.robot_pose_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, goal_topic, self.goal_callback, 10)

        # Publisher
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        # Stored grids
        self.costmap = None
        self.costmap_info = None
        self.map = None
        self.map_info = None
        self.robot_pose = None

        # PRM roadmap storage
        self.roadmap_nodes = []  # List of (x, y) in grid coordinates
        self.roadmap_edges = {}  # Dict: node_idx -> [(neighbor_idx, cost), ...]
        self.roadmap_built = False

        self.get_logger().info(
            f'PRM Planner Node started.\n'
            f'  num_samples: {self.num_samples}\n'
            f'  connection_radius: {self.connection_radius_m}m\n'
            f'  max_neighbors: {self.max_neighbors}\n'
            f'  inflation_radius: {self.inflation_radius_m}m\n'
            f'  safety_distance: {self.safety_distance_m}m\n'
            f'  rebuild_on_costmap: {self.rebuild_on_costmap}'
        )

    # ---------------- Grid callbacks ----------------

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = list(msg.data)
        self.costmap_info = msg.info
        self.update_cell_parameters(msg.info.resolution)
        
        # Rebuild roadmap if requested
        if self.rebuild_on_costmap:
            self.roadmap_built = False
            self.get_logger().info('Costmap updated, roadmap will be rebuilt on next query')
        
        self.get_logger().info(
            f'Costmap received: {msg.info.width}x{msg.info.height} res={msg.info.resolution}m/cell'
        )

    def map_callback(self, msg: OccupancyGrid):
        self.map = list(msg.data)
        self.map_info = msg.info
        if self.inflation_radius_cells is None:
            self.update_cell_parameters(msg.info.resolution)

    def robot_pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg

    def update_cell_parameters(self, resolution: float):
        """Convert meter-based parameters to cell counts"""
        self.inflation_radius_cells = max(1, int(self.inflation_radius_m / resolution))
        self.safety_distance_cells = max(1, int(self.safety_distance_m / resolution))
        self.connection_radius_cells = max(2, int(self.connection_radius_m / resolution))
        self.collision_check_cells = max(1, int(self.collision_check_resolution / resolution))
        
        self.get_logger().info(
            f'Parameters for resolution {resolution}m/cell:\n'
            f'  inflation: {self.inflation_radius_m}m → {self.inflation_radius_cells} cells\n'
            f'  safety: {self.safety_distance_m}m → {self.safety_distance_cells} cells\n'
            f'  connection: {self.connection_radius_m}m → {self.connection_radius_cells} cells'
        )

    def goal_callback(self, goal: PoseStamped):
        if self.robot_pose is None:
            self.get_logger().warn('No robot pose yet, cannot plan')
            return

        grid, info, grid_name = self.get_active_grid()
        if grid is None or info is None:
            self.get_logger().warn('No costmap/map yet, cannot plan')
            return

        # Build roadmap if not built yet
        if not self.roadmap_built:
            self.get_logger().info(f'Building PRM roadmap on {grid_name}...')
            start_time = time.time()
            self.build_roadmap(grid, info)
            build_time = time.time() - start_time
            self.get_logger().info(
                f'Roadmap built in {build_time:.3f}s: '
                f'{len(self.roadmap_nodes)} nodes, '
                f'{sum(len(edges) for edges in self.roadmap_edges.values())} edges'
            )

        self.get_logger().info('New goal received, planning with PRM...')
        start_time = time.time()
        path_msg = self.plan_prm(self.robot_pose, goal, grid, info)
        plan_time = time.time() - start_time
        
        if path_msg is None or len(path_msg.poses) == 0:
            self.get_logger().warn('PRM failed to find a path')
            return

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'Path found in {plan_time:.3f}s with {len(path_msg.poses)} waypoints'
        )

    def get_active_grid(self):
        if self.costmap is not None and self.costmap_info is not None:
            return self.costmap, self.costmap_info, 'costmap'
        return self.map, self.map_info, 'map'

    # ---------------- Grid utilities ----------------

    def world_to_map(self, x: float, y: float, info):
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution
        mx = int((x - origin_x) / res)
        my = int((y - origin_y) / res)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return mx, my

    def map_to_world(self, mx: int, my: int, info):
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution
        x = origin_x + (mx + 0.5) * res
        y = origin_y + (my + 0.5) * res
        return x, y

    def cell_value(self, mx: int, my: int, grid, info) -> int:
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return -1
        idx = my * info.width + mx
        return int(grid[idx])

    def is_blocked(self, mx: int, my: int, grid, info) -> bool:
        v = self.cell_value(mx, my, grid, info)
        if v < 0:
            return self.unknown_is_obstacle
        return v >= self.occupied_threshold

    def is_collision_free(self, mx: int, my: int, grid, info) -> bool:
        """Check if a cell and its safety radius are collision-free"""
        if self.safety_distance_cells is None:
            return not self.is_blocked(mx, my, grid, info)
        
        # Check the cell itself
        if self.is_blocked(mx, my, grid, info):
            return False
        
        # Check safety radius around the cell
        for dx in range(-self.safety_distance_cells, self.safety_distance_cells + 1):
            for dy in range(-self.safety_distance_cells, self.safety_distance_cells + 1):
                if dx == 0 and dy == 0:
                    continue
                dist = math.hypot(dx, dy)
                if dist <= self.safety_distance_cells:
                    if self.is_blocked(mx + dx, my + dy, grid, info):
                        return False
        return True

    def is_edge_collision_free(self, p1: Tuple[int, int], p2: Tuple[int, int], 
                                grid, info) -> bool:
        """Check if edge between two nodes is collision-free using interpolation"""
        x1, y1 = p1
        x2, y2 = p2
        
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist == 0:
            return True
        
        # Number of checks based on collision check resolution
        num_checks = max(2, int(dist / self.collision_check_cells))
        
        for i in range(num_checks + 1):
            t = i / num_checks
            x = int(x1 + t * (x2 - x1))
            y = int(y1 + t * (y2 - y1))
            
            if not self.is_collision_free(x, y, grid, info):
                return False
        
        return True

    def get_edge_cost(self, p1: Tuple[int, int], p2: Tuple[int, int], 
                      grid, info) -> float:
        """Calculate cost of edge including distance and terrain cost"""
        x1, y1 = p1
        x2, y2 = p2
        
        # Base distance cost
        dist = math.hypot(x2 - x1, y2 - y1)
        
        # Add terrain cost along the edge
        num_samples = max(2, int(dist / self.collision_check_cells))
        terrain_cost = 0.0
        
        for i in range(num_samples + 1):
            t = i / num_samples
            x = int(x1 + t * (x2 - x1))
            y = int(y1 + t * (y2 - y1))
            
            v = self.cell_value(x, y, grid, info)
            if v >= 0:
                terrain_cost += (v / 100.0) * self.cost_weight
        
        terrain_cost /= num_samples
        
        return dist + terrain_cost

    # ---------------- PRM Implementation ----------------

    def build_roadmap(self, grid, info):
        """Build the PRM roadmap with random sampling and connections"""
        self.roadmap_nodes = []
        self.roadmap_edges = {}
        
        width = info.width
        height = info.height
        
        # Sample random collision-free nodes
        samples_generated = 0
        max_attempts = self.num_samples * 10
        
        while len(self.roadmap_nodes) < self.num_samples and samples_generated < max_attempts:
            # Random sample in grid space
            x = random.randint(0, width - 1)
            y = random.randint(0, height - 1)
            
            samples_generated += 1
            
            if self.is_collision_free(x, y, grid, info):
                self.roadmap_nodes.append((x, y))
        
        if len(self.roadmap_nodes) < self.num_samples:
            self.get_logger().warn(
                f'Only sampled {len(self.roadmap_nodes)}/{self.num_samples} valid nodes'
            )
        
        # Build KD-tree for efficient nearest neighbor search (using simple approach)
        # Connect nodes within connection radius
        for i, node in enumerate(self.roadmap_nodes):
            self.roadmap_edges[i] = []
            
            # Find neighbors within connection radius
            neighbors = []
            for j, other_node in enumerate(self.roadmap_nodes):
                if i == j:
                    continue
                
                dist = math.hypot(node[0] - other_node[0], node[1] - other_node[1])
                if dist <= self.connection_radius_cells:
                    neighbors.append((j, dist))
            
            # Sort by distance and take closest max_neighbors
            neighbors.sort(key=lambda x: x[1])
            neighbors = neighbors[:self.max_neighbors]
            
            # Check collision for each edge and add if free
            for j, dist in neighbors:
                if self.is_edge_collision_free(node, self.roadmap_nodes[j], grid, info):
                    cost = self.get_edge_cost(node, self.roadmap_nodes[j], grid, info)
                    self.roadmap_edges[i].append((j, cost))
        
        self.roadmap_built = True

    def find_nearest_node(self, point: Tuple[int, int]) -> Optional[int]:
        """Find the nearest roadmap node to a given point"""
        if not self.roadmap_nodes:
            return None
        
        min_dist = float('inf')
        nearest_idx = None
        
        for i, node in enumerate(self.roadmap_nodes):
            dist = math.hypot(point[0] - node[0], point[1] - node[1])
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx

    def plan_prm(self, start_pose: PoseStamped, goal_pose: PoseStamped, 
                 grid, info) -> Optional[Path]:
        """Plan path using pre-built PRM roadmap"""
        sx = start_pose.pose.position.x
        sy = start_pose.pose.position.y
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        start_idx = self.world_to_map(sx, sy, info)
        goal_idx = self.world_to_map(gx, gy, info)

        if start_idx is None or goal_idx is None:
            self.get_logger().warn('Start or goal outside map bounds')
            return None

        # Check if start and goal are collision-free
        if not self.is_collision_free(start_idx[0], start_idx[1], grid, info):
            self.get_logger().warn('Start position is in collision')
            return None
        
        if not self.is_collision_free(goal_idx[0], goal_idx[1], grid, info):
            self.get_logger().warn('Goal position is in collision')
            return None

        # Add start and goal to roadmap temporarily
        start_node_idx = len(self.roadmap_nodes)
        goal_node_idx = len(self.roadmap_nodes) + 1
        
        temp_nodes = self.roadmap_nodes + [start_idx, goal_idx]
        temp_edges = dict(self.roadmap_edges)
        
        # Connect start to nearby roadmap nodes
        temp_edges[start_node_idx] = []
        for i, node in enumerate(self.roadmap_nodes):
            dist = math.hypot(start_idx[0] - node[0], start_idx[1] - node[1])
            if dist <= self.connection_radius_cells:
                if self.is_edge_collision_free(start_idx, node, grid, info):
                    cost = self.get_edge_cost(start_idx, node, grid, info)
                    temp_edges[start_node_idx].append((i, cost))
        
        # Connect goal to nearby roadmap nodes
        temp_edges[goal_node_idx] = []
        for i, node in enumerate(self.roadmap_nodes):
            dist = math.hypot(goal_idx[0] - node[0], goal_idx[1] - node[1])
            if dist <= self.connection_radius_cells:
                if self.is_edge_collision_free(goal_idx, node, grid, info):
                    cost = self.get_edge_cost(goal_idx, node, grid, info)
                    temp_edges[goal_node_idx].append((i, cost))
                    # Add reverse edge
                    if i not in temp_edges:
                        temp_edges[i] = list(self.roadmap_edges.get(i, []))
                    temp_edges[i].append((goal_node_idx, cost))
        
        # Run A* on the roadmap
        path_indices = self.a_star_roadmap(
            start_node_idx, goal_node_idx, temp_nodes, temp_edges
        )
        
        if path_indices is None:
            return None
        
        # Convert path indices to world coordinates
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = start_pose.header.frame_id
        
        for idx in path_indices:
            mx, my = temp_nodes[idx]
            x, y = self.map_to_world(mx, my, info)
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        # Smooth path if enabled
        if self.smooth_path and len(path.poses) > 2:
            path = self.smooth_path_shortcut(path, grid, info)
        
        return path

    def a_star_roadmap(self, start_idx: int, goal_idx: int, 
                       nodes: List[Tuple[int, int]], 
                       edges: dict) -> Optional[List[int]]:
        """A* search on the roadmap graph"""
        def heuristic(idx1: int, idx2: int) -> float:
            x1, y1 = nodes[idx1]
            x2, y2 = nodes[idx2]
            return math.hypot(x2 - x1, y2 - y1)
        
        open_set = []
        heapq.heappush(open_set, (0.0, start_idx))
        came_from = {}
        g_score = {start_idx: 0.0}
        visited = set()
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current in visited:
                continue
            visited.add(current)
            
            if current == goal_idx:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            # Explore neighbors
            if current not in edges:
                continue
            
            for neighbor, cost in edges[current]:
                if neighbor in visited:
                    continue
                
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal_idx)
                    heapq.heappush(open_set, (f_score, neighbor))
        
        return None

    def smooth_path_shortcut(self, path: Path, grid, info) -> Path:
        """Smooth path using shortcut method"""
        if len(path.poses) <= 2:
            return path
        
        smoothed = Path()
        smoothed.header = path.header
        smoothed.poses.append(path.poses[0])
        
        current_idx = 0
        
        while current_idx < len(path.poses) - 1:
            # Try to connect to farthest visible waypoint
            farthest_idx = current_idx + 1
            
            for test_idx in range(len(path.poses) - 1, current_idx, -1):
                p1 = self.world_to_map(
                    path.poses[current_idx].pose.position.x,
                    path.poses[current_idx].pose.position.y,
                    info
                )
                p2 = self.world_to_map(
                    path.poses[test_idx].pose.position.x,
                    path.poses[test_idx].pose.position.y,
                    info
                )
                
                if p1 and p2 and self.is_edge_collision_free(p1, p2, grid, info):
                    farthest_idx = test_idx
                    break
            
            smoothed.poses.append(path.poses[farthest_idx])
            current_idx = farthest_idx
        
        self.get_logger().info(
            f'Path smoothed: {len(path.poses)} → {len(smoothed.poses)} waypoints'
        )
        return smoothed


def main(args=None):
    rclpy.init(args=args)
    node = PRMPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()