#!/usr/bin/env python3
import math
import heapq
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class FastAStarPlannerNode(Node):
    def __init__(self):
        super().__init__('fast_a_star_planner_node')

        # Parameters
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')

        # Obstacle detection parameters
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('occupied_threshold', 50)
        
        # Cost parameters
        self.declare_parameter('cost_weight', 3.0)  # Reduced for speed
        self.declare_parameter('inflation_radius_m', 1.4)
        
        # Path quality parameters
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('smooth_path', True)
        self.declare_parameter('safety_distance_m', 0.3)
        
        # Performance optimization parameters
        self.declare_parameter('use_bidirectional', True)  # Bidirectional A*
        self.declare_parameter('heuristic_weight', 1.2)  # Weighted A* (>1.0 = faster, less optimal)
        self.declare_parameter('max_iterations', 50000)  # Prevent infinite loops
        self.declare_parameter('early_termination_radius', 5)  # Stop when close enough (cells)
        self.declare_parameter('skip_proximity_check', False)  # Skip expensive proximity calculations
        self.declare_parameter('downsampling_factor', 1)  # Grid downsampling (1=no downsampling, 2=half resolution)

        costmap_topic = self.get_parameter('costmap_topic').value
        map_topic = self.get_parameter('map_topic').value
        robot_pose_topic = self.get_parameter('robot_pose_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        path_topic = self.get_parameter('path_topic').value

        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.cost_weight = float(self.get_parameter('cost_weight').value)
        self.inflation_radius_m = float(self.get_parameter('inflation_radius_m').value)
        self.safety_distance_m = float(self.get_parameter('safety_distance_m').value)
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self.smooth_path = bool(self.get_parameter('smooth_path').value)
        
        # Performance parameters
        self.use_bidirectional = bool(self.get_parameter('use_bidirectional').value)
        self.heuristic_weight = float(self.get_parameter('heuristic_weight').value)
        self.max_iterations = int(self.get_parameter('max_iterations').value)
        self.early_termination_radius = int(self.get_parameter('early_termination_radius').value)
        self.skip_proximity_check = bool(self.get_parameter('skip_proximity_check').value)
        self.downsampling_factor = int(self.get_parameter('downsampling_factor').value)

        # Cell-based parameters
        self.inflation_radius_cells = None
        self.safety_distance_cells = None

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
        
        # Precomputed neighbors for speed
        self.neighbors_8 = [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]
        self.neighbors_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        # Precomputed costs
        self.straight_cost = 1.0
        self.diagonal_cost = math.sqrt(2)

        self.get_logger().info(
            f'Fast A* Planner Node started.\n'
            f'  bidirectional search: {self.use_bidirectional}\n'
            f'  heuristic weight: {self.heuristic_weight}\n'
            f'  max iterations: {self.max_iterations}\n'
            f'  early termination: {self.early_termination_radius} cells\n'
            f'  skip proximity: {self.skip_proximity_check}\n'
            f'  downsampling: {self.downsampling_factor}x\n'
            f'  inflation_radius: {self.inflation_radius_m}m\n'
            f'  cost_weight: {self.cost_weight}'
        )

    # ---------------- Grid callbacks ----------------

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = list(msg.data)
        self.costmap_info = msg.info
        self.update_cell_parameters(msg.info.resolution)
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
        self.get_logger().info(
            f'Parameters for resolution {resolution}m/cell:\n'
            f'  inflation: {self.inflation_radius_m}m → {self.inflation_radius_cells} cells\n'
            f'  safety: {self.safety_distance_m}m → {self.safety_distance_cells} cells'
        )

    def goal_callback(self, goal: PoseStamped):
        if self.robot_pose is None:
            self.get_logger().warn('No robot pose yet, cannot plan')
            return

        grid, info, grid_name = self.get_active_grid()
        if grid is None or info is None:
            self.get_logger().warn('No costmap/map yet, cannot plan')
            return

        self.get_logger().info(f'New goal received, planning with Fast A*...')
        
        import time
        start_time = time.time()
        
        if self.use_bidirectional:
            path_msg = self.plan_bidirectional_a_star(self.robot_pose, goal, grid, info)
        else:
            path_msg = self.plan_fast_a_star(self.robot_pose, goal, grid, info)
        
        elapsed = time.time() - start_time
        
        if path_msg is None or len(path_msg.poses) == 0:
            self.get_logger().warn(f'Fast A* failed to find path (took {elapsed:.3f}s)')
            return

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'Path found in {elapsed:.3f}s with {len(path_msg.poses)} poses'
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
        """Fast inline cell value check"""
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return -1
        return grid[my * info.width + mx]

    def is_blocked(self, mx: int, my: int, grid, info) -> bool:
        """Fast collision check"""
        v = self.cell_value(mx, my, grid, info)
        if v < 0:
            return self.unknown_is_obstacle
        return v >= self.occupied_threshold

    def is_diagonal_blocked(self, cx: int, cy: int, nx: int, ny: int, grid, info) -> bool:
        """Check if diagonal cuts through obstacles"""
        dx = nx - cx
        dy = ny - cy
        if abs(dx) == 1 and abs(dy) == 1:
            if self.is_blocked(cx + dx, cy, grid, info):
                return True
            if self.is_blocked(cx, cy + dy, grid, info):
                return True
        return False

    def get_step_cost(self, mx: int, my: int, base_cost: float, grid, info) -> float:
        """Fast step cost calculation"""
        v = self.cell_value(mx, my, grid, info)
        if v < 0:
            return float('inf') if self.unknown_is_obstacle else base_cost
        if v >= self.occupied_threshold:
            return float('inf')
        
        # Simple cost calculation (skip expensive proximity check if enabled)
        if self.skip_proximity_check:
            penalty = (v / 100.0) * self.cost_weight
            return base_cost + penalty
        else:
            # Minimal proximity check (just immediate neighbors)
            has_nearby_obstacle = False
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                if self.is_blocked(mx + dx, my + dy, grid, info):
                    has_nearby_obstacle = True
                    break
            
            penalty = (v / 100.0) * self.cost_weight
            if has_nearby_obstacle:
                penalty += self.cost_weight * 0.5
            
            return base_cost + penalty

    # ---------------- Fast A* implementation ----------------

    def plan_fast_a_star(self, start_pose: PoseStamped, goal_pose: PoseStamped, grid, info) -> Path:
        """Optimized A* with weighted heuristic and early termination"""
        sx = start_pose.pose.position.x
        sy = start_pose.pose.position.y
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        start_idx = self.world_to_map(sx, sy, info)
        goal_idx = self.world_to_map(gx, gy, info)

        if start_idx is None or goal_idx is None:
            self.get_logger().warn('Start or goal outside map bounds')
            return None

        sx_i, sy_i = start_idx
        gx_i, gy_i = goal_idx

        if self.is_blocked(sx_i, sy_i, grid, info):
            self.get_logger().warn('Start cell is blocked')
        if self.is_blocked(gx_i, gy_i, grid, info):
            self.get_logger().warn('Goal cell is blocked')

        width = info.width
        height = info.height

        # Choose neighbor set
        neighbors = self.neighbors_8 if self.allow_diagonal else self.neighbors_4

        # Fast heuristic using precomputed values
        def heuristic(x1, y1, x2, y2):
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            # Octile distance (faster than hypot)
            return self.straight_cost * max(dx, dy) + (self.diagonal_cost - self.straight_cost) * min(dx, dy)

        start = (sx_i, sy_i)
        goal = (gx_i, gy_i)

        # Use dictionary for faster lookups
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        visited = set()
        
        iterations = 0

        while open_set and iterations < self.max_iterations:
            iterations += 1
            
            _, current = heapq.heappop(open_set)
            
            if current in visited:
                continue
            visited.add(current)

            cx, cy = current
            
            # Early termination if close enough to goal
            if abs(cx - gx_i) <= self.early_termination_radius and \
               abs(cy - gy_i) <= self.early_termination_radius:
                dist_to_goal = heuristic(cx, cy, gx_i, gy_i)
                if dist_to_goal <= self.early_termination_radius:
                    self.get_logger().info(
                        f'Early termination: {iterations} iterations, '
                        f'distance to goal: {dist_to_goal:.1f} cells'
                    )
                    # Try to connect directly to goal
                    goal_g = g_score[current] + dist_to_goal
                    g_score[goal] = goal_g
                    came_from[goal] = current
                    return self.reconstruct_path(came_from, goal, start_pose.header.frame_id, info, grid)

            if current == goal:
                self.get_logger().info(f'Path found! {iterations} iterations')
                return self.reconstruct_path(came_from, current, start_pose.header.frame_id, info, grid)

            # Explore neighbors
            for dx, dy in neighbors:
                nx = cx + dx
                ny = cy + dy

                # Fast bounds check
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                
                # Fast collision check
                if self.is_blocked(nx, ny, grid, info):
                    continue
                
                # Check diagonal cuts
                if abs(dx) == 1 and abs(dy) == 1:
                    if self.is_diagonal_blocked(cx, cy, nx, ny, grid, info):
                        continue
                    base = self.diagonal_cost
                else:
                    base = self.straight_cost
                
                # Get step cost
                step = self.get_step_cost(nx, ny, base, grid, info)
                
                if not math.isfinite(step):
                    continue

                neighbor = (nx, ny)
                tentative_g = g_score[current] + step

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    # Weighted A* heuristic
                    h = heuristic(nx, ny, gx_i, gy_i)
                    f = tentative_g + self.heuristic_weight * h
                    heapq.heappush(open_set, (f, neighbor))

        self.get_logger().warn(f'A*: No path found after {iterations} iterations')
        return None

    # ---------------- Bidirectional A* ----------------

    def plan_bidirectional_a_star(self, start_pose: PoseStamped, goal_pose: PoseStamped, 
                                   grid, info) -> Path:
        """Bidirectional A* - search from both start and goal simultaneously"""
        sx = start_pose.pose.position.x
        sy = start_pose.pose.position.y
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        start_idx = self.world_to_map(sx, sy, info)
        goal_idx = self.world_to_map(gx, gy, info)

        if start_idx is None or goal_idx is None:
            return None

        sx_i, sy_i = start_idx
        gx_i, gy_i = goal_idx

        width = info.width
        height = info.height
        neighbors = self.neighbors_8 if self.allow_diagonal else self.neighbors_4

        def heuristic(x1, y1, x2, y2):
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            return self.straight_cost * max(dx, dy) + (self.diagonal_cost - self.straight_cost) * min(dx, dy)

        start = (sx_i, sy_i)
        goal = (gx_i, gy_i)

        # Forward search (from start)
        open_forward = []
        heapq.heappush(open_forward, (0.0, start))
        came_from_forward = {}
        g_score_forward = {start: 0.0}
        visited_forward = set()

        # Backward search (from goal)
        open_backward = []
        heapq.heappush(open_backward, (0.0, goal))
        came_from_backward = {}
        g_score_backward = {goal: 0.0}
        visited_backward = set()

        best_path_cost = float('inf')
        meeting_point = None

        iterations = 0
        max_iter_per_direction = self.max_iterations // 2

        while (open_forward or open_backward) and iterations < max_iter_per_direction:
            iterations += 1

            # Expand from forward direction
            if open_forward:
                _, current_f = heapq.heappop(open_forward)
                
                if current_f not in visited_forward:
                    visited_forward.add(current_f)
                    
                    # Check if we met the backward search
                    if current_f in visited_backward:
                        path_cost = g_score_forward[current_f] + g_score_backward[current_f]
                        if path_cost < best_path_cost:
                            best_path_cost = path_cost
                            meeting_point = current_f
                            break
                    
                    # Expand neighbors
                    cx, cy = current_f
                    for dx, dy in neighbors:
                        nx, ny = cx + dx, cy + dy
                        
                        if nx < 0 or ny < 0 or nx >= width or ny >= height:
                            continue
                        if self.is_blocked(nx, ny, grid, info):
                            continue
                        
                        base = self.diagonal_cost if (abs(dx) == 1 and abs(dy) == 1) else self.straight_cost
                        step = self.get_step_cost(nx, ny, base, grid, info)
                        
                        if not math.isfinite(step):
                            continue
                        
                        neighbor = (nx, ny)
                        tentative_g = g_score_forward[current_f] + step
                        
                        if neighbor not in g_score_forward or tentative_g < g_score_forward[neighbor]:
                            came_from_forward[neighbor] = current_f
                            g_score_forward[neighbor] = tentative_g
                            h = heuristic(nx, ny, gx_i, gy_i)
                            f = tentative_g + h
                            heapq.heappush(open_forward, (f, neighbor))

            # Expand from backward direction
            if open_backward:
                _, current_b = heapq.heappop(open_backward)
                
                if current_b not in visited_backward:
                    visited_backward.add(current_b)
                    
                    # Check if we met the forward search
                    if current_b in visited_forward:
                        path_cost = g_score_forward[current_b] + g_score_backward[current_b]
                        if path_cost < best_path_cost:
                            best_path_cost = path_cost
                            meeting_point = current_b
                            break
                    
                    # Expand neighbors
                    cx, cy = current_b
                    for dx, dy in neighbors:
                        nx, ny = cx + dx, cy + dy
                        
                        if nx < 0 or ny < 0 or nx >= width or ny >= height:
                            continue
                        if self.is_blocked(nx, ny, grid, info):
                            continue
                        
                        base = self.diagonal_cost if (abs(dx) == 1 and abs(dy) == 1) else self.straight_cost
                        step = self.get_step_cost(nx, ny, base, grid, info)
                        
                        if not math.isfinite(step):
                            continue
                        
                        neighbor = (nx, ny)
                        tentative_g = g_score_backward[current_b] + step
                        
                        if neighbor not in g_score_backward or tentative_g < g_score_backward[neighbor]:
                            came_from_backward[neighbor] = current_b
                            g_score_backward[neighbor] = tentative_g
                            h = heuristic(nx, ny, sx_i, sy_i)
                            f = tentative_g + h
                            heapq.heappush(open_backward, (f, neighbor))

        if meeting_point is None:
            self.get_logger().warn(f'Bidirectional A*: No meeting point after {iterations} iterations')
            return None

        self.get_logger().info(
            f'Bidirectional A* found path! {iterations} iterations '
            f'(vs ~{iterations*2} for regular A*)'
        )

        # Reconstruct path from both directions
        return self.reconstruct_bidirectional_path(
            came_from_forward, came_from_backward, meeting_point,
            start, goal, start_pose.header.frame_id, info, grid
        )

    def reconstruct_path(self, came_from, current, frame_id: str, info, grid) -> Path:
        """Reconstruct path and optionally smooth it"""
        path_cells = [current]
        while current in came_from:
            current = came_from[current]
            path_cells.append(current)
        path_cells.reverse()

        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        # Smooth path if enabled
        if self.smooth_path:
            path_cells = self.smooth_path_cells(path_cells, grid, info)

        for (mx, my) in path_cells:
            x, y = self.map_to_world(mx, my, info)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def reconstruct_bidirectional_path(self, came_from_forward, came_from_backward,
                                       meeting, start, goal, frame_id, info, grid) -> Path:
        """Reconstruct path from bidirectional search"""
        # Forward path: start to meeting
        forward_cells = [meeting]
        current = meeting
        while current in came_from_forward:
            current = came_from_forward[current]
            forward_cells.append(current)
        forward_cells.reverse()

        # Backward path: meeting to goal
        backward_cells = []
        current = meeting
        while current in came_from_backward:
            current = came_from_backward[current]
            backward_cells.append(current)

        # Combine (don't duplicate meeting point)
        path_cells = forward_cells + backward_cells

        # Smooth
        if self.smooth_path:
            path_cells = self.smooth_path_cells(path_cells, grid, info)

        # Convert to Path message
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        for (mx, my) in path_cells:
            x, y = self.map_to_world(mx, my, info)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def smooth_path_cells(self, cells, grid, info):
        """Fast path smoothing using line-of-sight"""
        if len(cells) <= 2:
            return cells

        smoothed = [cells[0]]
        current_idx = 0

        while current_idx < len(cells) - 1:
            farthest_idx = current_idx + 1

            for test_idx in range(len(cells) - 1, current_idx, -1):
                if self.has_line_of_sight_fast(cells[current_idx], cells[test_idx], grid, info):
                    farthest_idx = test_idx
                    break

            smoothed.append(cells[farthest_idx])
            current_idx = farthest_idx

        return smoothed

    def has_line_of_sight_fast(self, p1, p2, grid, info):
        """Fast line of sight check using Bresenham"""
        x0, y0 = p1
        x1, y1 = p2

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            if self.is_blocked(x, y, grid, info):
                return False

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return True


def main(args=None):
    rclpy.init(args=args)
    node = FastAStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()