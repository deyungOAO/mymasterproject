#!/usr/bin/env python3
"""
Fast A* / Bidirectional A* Global Planner
- Smooth path using line-of-sight
- Densify path so the controller always has plenty of waypoints
- Each waypoint carries a heading (yaw) computed from direction of travel
"""
import math
import heapq

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class FastAStarPlannerNode(Node):
    def __init__(self):
        super().__init__('fast_a_star_planner_node')

        # ── Topic parameters ──────────────────────────────────────────────────
        self.declare_parameter('costmap_topic',    '/global_costmap/costmap')
        self.declare_parameter('map_topic',        '/map')
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('goal_topic',       '/goal_pose')
        self.declare_parameter('path_topic',       '/planned_path')

        # ── Obstacle / cost parameters ────────────────────────────────────────
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('occupied_threshold',  50)
        self.declare_parameter('cost_weight',         3.0)
        self.declare_parameter('inflation_radius_m',  1.4)

        # ── Path quality parameters ───────────────────────────────────────────
        self.declare_parameter('allow_diagonal',      True)
        self.declare_parameter('smooth_path',         True)
        self.declare_parameter('safety_distance_m',   0.3)

        # ── Waypoint densification ────────────────────────────────────────────
        # After smoothing, interpolate so waypoints are at most
        # `waypoint_spacing` metres apart.  Set to 0.0 to disable.
        self.declare_parameter('waypoint_spacing', 5)#0.4

        # ── Performance parameters ────────────────────────────────────────────
        self.declare_parameter('use_bidirectional',        True)
        self.declare_parameter('heuristic_weight',         1.2)
        self.declare_parameter('max_iterations',           50000)
        self.declare_parameter('early_termination_radius', 5)
        self.declare_parameter('skip_proximity_check',     False)
        self.declare_parameter('downsampling_factor',      1)

        # ── Read all params ───────────────────────────────────────────────────
        costmap_topic    = self.get_parameter('costmap_topic').value
        map_topic        = self.get_parameter('map_topic').value
        robot_pose_topic = self.get_parameter('robot_pose_topic').value
        goal_topic       = self.get_parameter('goal_topic').value
        path_topic       = self.get_parameter('path_topic').value

        self.unknown_is_obstacle      = bool(self.get_parameter('unknown_is_obstacle').value)
        self.occupied_threshold       = int(self.get_parameter('occupied_threshold').value)
        self.cost_weight              = float(self.get_parameter('cost_weight').value)
        self.inflation_radius_m       = float(self.get_parameter('inflation_radius_m').value)
        self.safety_distance_m        = float(self.get_parameter('safety_distance_m').value)
        self.allow_diagonal           = bool(self.get_parameter('allow_diagonal').value)
        self.smooth_path              = bool(self.get_parameter('smooth_path').value)
        self.waypoint_spacing         = float(self.get_parameter('waypoint_spacing').value)
        self.use_bidirectional        = bool(self.get_parameter('use_bidirectional').value)
        self.heuristic_weight         = float(self.get_parameter('heuristic_weight').value)
        self.max_iterations           = int(self.get_parameter('max_iterations').value)
        self.early_termination_radius = int(self.get_parameter('early_termination_radius').value)
        self.skip_proximity_check     = bool(self.get_parameter('skip_proximity_check').value)
        self.downsampling_factor      = int(self.get_parameter('downsampling_factor').value)

        # ── Cell-based params (set once we know resolution) ───────────────────
        self.inflation_radius_cells = None
        self.safety_distance_cells  = None

        # ── QoS for latched OccupancyGrid ─────────────────────────────────────
        grid_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(OccupancyGrid, costmap_topic,
                                 self.costmap_callback, grid_qos)
        self.create_subscription(OccupancyGrid, map_topic,
                                 self.map_callback, grid_qos)
        self.create_subscription(PoseStamped, robot_pose_topic,
                                 self.robot_pose_callback, 10)
        self.create_subscription(PoseStamped, goal_topic,
                                 self.goal_callback, 10)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        # ── State ─────────────────────────────────────────────────────────────
        self.costmap      = None
        self.costmap_info = None
        self.map          = None
        self.map_info     = None
        self.robot_pose   = None
        self.current_goal = None

        # ── Pre-computed neighbor offsets ─────────────────────────────────────
        self.neighbors_8 = [(-1, 0), (1, 0), (0, -1), (0, 1),
                            (-1, -1), (-1, 1), (1, -1), (1, 1)]
        self.neighbors_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        self.straight_cost = 1.0
        self.diagonal_cost = math.sqrt(2)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Fast A* Planner — dense waypoint mode')
        self.get_logger().info(f'  Bidirectional : {self.use_bidirectional}')
        self.get_logger().info(f'  Smooth path   : {self.smooth_path}')
        self.get_logger().info(f'  Waypoint spacing: {self.waypoint_spacing} m '
                               f'(0 = raw grid cells)')
        self.get_logger().info('=' * 60)

    # ─────────────────────────────────────────────────────────────────────────
    # Grid callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap      = list(msg.data)
        self.costmap_info = msg.info
        self.update_cell_parameters(msg.info.resolution)
        self.get_logger().info(
            f'Costmap received: {msg.info.width}x{msg.info.height} '
            f'@ {msg.info.resolution} m/cell')

    def map_callback(self, msg: OccupancyGrid):
        self.map      = list(msg.data)
        self.map_info = msg.info
        if self.inflation_radius_cells is None:
            self.update_cell_parameters(msg.info.resolution)

    def robot_pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg

    def update_cell_parameters(self, resolution: float):
        self.inflation_radius_cells = max(1, int(self.inflation_radius_m / resolution))
        self.safety_distance_cells  = max(1, int(self.safety_distance_m / resolution))

    def goal_callback(self, goal: PoseStamped):
        if self.robot_pose is None:
            self.get_logger().warn('No robot pose yet — cannot plan')
            return

        grid, info, _ = self.get_active_grid()
        if grid is None or info is None:
            self.get_logger().warn('No map/costmap yet — cannot plan')
            return

        self.current_goal = goal
        self.get_logger().info('New goal received — planning...')

        import time
        t0 = time.time()

        if self.use_bidirectional:
            path_msg = self.plan_bidirectional_a_star(
                self.robot_pose, goal, grid, info)
        else:
            path_msg = self.plan_fast_a_star(
                self.robot_pose, goal, grid, info)

        elapsed = time.time() - t0

        if path_msg is None or len(path_msg.poses) == 0:
            self.get_logger().warn(f'Path planning failed ({elapsed:.3f}s)')
            return

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'✔ Path published: {len(path_msg.poses)} waypoints ({elapsed:.3f}s)')

    # ─────────────────────────────────────────────────────────────────────────
    # Grid utilities
    # ─────────────────────────────────────────────────────────────────────────

    def get_active_grid(self):
        if self.costmap is not None and self.costmap_info is not None:
            return self.costmap, self.costmap_info, 'costmap'
        return self.map, self.map_info, 'map'

    def world_to_map(self, x, y, info):
        ox  = info.origin.position.x
        oy  = info.origin.position.y
        res = info.resolution
        mx  = int((x - ox) / res)
        my  = int((y - oy) / res)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return mx, my

    def map_to_world(self, mx, my, info):
        ox  = info.origin.position.x
        oy  = info.origin.position.y
        res = info.resolution
        return ox + (mx + 0.5) * res, oy + (my + 0.5) * res

    def cell_value(self, mx, my, grid, info):
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return -1
        return grid[my * info.width + mx]

    def is_blocked(self, mx, my, grid, info):
        v = self.cell_value(mx, my, grid, info)
        if v < 0:
            return self.unknown_is_obstacle
        return v >= self.occupied_threshold

    def is_diagonal_blocked(self, cx, cy, nx, ny, grid, info):
        dx = nx - cx
        dy = ny - cy
        if abs(dx) == 1 and abs(dy) == 1:
            if self.is_blocked(cx + dx, cy, grid, info):
                return True
            if self.is_blocked(cx, cy + dy, grid, info):
                return True
        return False

    def get_step_cost(self, mx, my, base_cost, grid, info):
        v = self.cell_value(mx, my, grid, info)
        if v < 0:
            return float('inf') if self.unknown_is_obstacle else base_cost
        if v >= self.occupied_threshold:
            return float('inf')
        if self.skip_proximity_check:
            return base_cost + (v / 100.0) * self.cost_weight
        has_nearby = any(
            self.is_blocked(mx + dx, my + dy, grid, info)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        )
        penalty = (v / 100.0) * self.cost_weight
        if has_nearby:
            penalty += self.cost_weight * 0.5
        return base_cost + penalty

    # ─────────────────────────────────────────────────────────────────────────
    # A* search
    # ─────────────────────────────────────────────────────────────────────────

    def plan_fast_a_star(self, start_pose, goal_pose, grid, info):
        sx_i, sy_i = self.world_to_map(
            start_pose.pose.position.x, start_pose.pose.position.y, info) or (None, None)
        gx_i, gy_i = self.world_to_map(
            goal_pose.pose.position.x, goal_pose.pose.position.y, info) or (None, None)

        if sx_i is None or gx_i is None:
            return None

        width     = info.width
        height    = info.height
        neighbors = self.neighbors_8 if self.allow_diagonal else self.neighbors_4

        def heuristic(x1, y1, x2, y2):
            dx = abs(x2 - x1); dy = abs(y2 - y1)
            return (self.straight_cost * max(dx, dy) +
                    (self.diagonal_cost - self.straight_cost) * min(dx, dy))

        start = (sx_i, sy_i)
        goal  = (gx_i, gy_i)

        open_set  = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score   = {start: 0.0}
        visited   = set()
        iters     = 0

        while open_set and iters < self.max_iterations:
            iters += 1
            _, current = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)

            cx, cy = current
            if (abs(cx - gx_i) <= self.early_termination_radius and
                    abs(cy - gy_i) <= self.early_termination_radius):
                d = heuristic(cx, cy, gx_i, gy_i)
                if d <= self.early_termination_radius:
                    came_from[goal] = current
                    return self._build_path(came_from, goal,
                                            start_pose.header.frame_id, info, grid)

            if current == goal:
                return self._build_path(came_from, current,
                                        start_pose.header.frame_id, info, grid)

            for dx, dy in neighbors:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                if self.is_blocked(nx, ny, grid, info):
                    continue
                if abs(dx) == 1 and abs(dy) == 1:
                    if self.is_diagonal_blocked(cx, cy, nx, ny, grid, info):
                        continue
                    base = self.diagonal_cost
                else:
                    base = self.straight_cost

                step = self.get_step_cost(nx, ny, base, grid, info)
                if not math.isfinite(step):
                    continue

                neighbor     = (nx, ny)
                tentative_g  = g_score[current] + step
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor]   = tentative_g
                    f = tentative_g + self.heuristic_weight * heuristic(nx, ny, gx_i, gy_i)
                    heapq.heappush(open_set, (f, neighbor))

        return None

    def plan_bidirectional_a_star(self, start_pose, goal_pose, grid, info):
        sx_i, sy_i = self.world_to_map(
            start_pose.pose.position.x, start_pose.pose.position.y, info) or (None, None)
        gx_i, gy_i = self.world_to_map(
            goal_pose.pose.position.x, goal_pose.pose.position.y, info) or (None, None)

        if sx_i is None or gx_i is None:
            return None

        width     = info.width
        height    = info.height
        neighbors = self.neighbors_8 if self.allow_diagonal else self.neighbors_4

        def heuristic(x1, y1, x2, y2):
            dx = abs(x2 - x1); dy = abs(y2 - y1)
            return (self.straight_cost * max(dx, dy) +
                    (self.diagonal_cost - self.straight_cost) * min(dx, dy))

        start = (sx_i, sy_i)
        goal  = (gx_i, gy_i)

        open_f    = []; heapq.heappush(open_f, (0.0, start))
        cf        = {}
        gf        = {start: 0.0}
        vis_f     = set()

        open_b    = []; heapq.heappush(open_b, (0.0, goal))
        cb        = {}
        gb        = {goal: 0.0}
        vis_b     = set()

        best_cost    = float('inf')
        meeting_pt   = None
        max_per_dir  = self.max_iterations // 2
        iters        = 0

        while (open_f or open_b) and iters < max_per_dir:
            iters += 1

            # ── Forward step ──────────────────────────────────────────────────
            if open_f:
                _, cur = heapq.heappop(open_f)
                if cur not in vis_f:
                    vis_f.add(cur)
                    if cur in vis_b:
                        c = gf[cur] + gb[cur]
                        if c < best_cost:
                            best_cost  = c
                            meeting_pt = cur
                            break
                    cx, cy = cur
                    for dx, dy in neighbors:
                        nx, ny = cx + dx, cy + dy
                        if nx < 0 or ny < 0 or nx >= width or ny >= height:
                            continue
                        if self.is_blocked(nx, ny, grid, info):
                            continue
                        base = self.diagonal_cost if (abs(dx)==1 and abs(dy)==1) else self.straight_cost
                        step = self.get_step_cost(nx, ny, base, grid, info)
                        if not math.isfinite(step):
                            continue
                        nb = (nx, ny)
                        tg = gf[cur] + step
                        if nb not in gf or tg < gf[nb]:
                            cf[nb] = cur
                            gf[nb] = tg
                            heapq.heappush(open_f,
                                (tg + heuristic(nx, ny, gx_i, gy_i), nb))

            # ── Backward step ─────────────────────────────────────────────────
            if open_b:
                _, cur = heapq.heappop(open_b)
                if cur not in vis_b:
                    vis_b.add(cur)
                    if cur in vis_f:
                        c = gf[cur] + gb[cur]
                        if c < best_cost:
                            best_cost  = c
                            meeting_pt = cur
                            break
                    cx, cy = cur
                    for dx, dy in neighbors:
                        nx, ny = cx + dx, cy + dy
                        if nx < 0 or ny < 0 or nx >= width or ny >= height:
                            continue
                        if self.is_blocked(nx, ny, grid, info):
                            continue
                        base = self.diagonal_cost if (abs(dx)==1 and abs(dy)==1) else self.straight_cost
                        step = self.get_step_cost(nx, ny, base, grid, info)
                        if not math.isfinite(step):
                            continue
                        nb = (nx, ny)
                        tg = gb[cur] + step
                        if nb not in gb or tg < gb[nb]:
                            cb[nb] = cur
                            gb[nb] = tg
                            heapq.heappush(open_b,
                                (tg + heuristic(nx, ny, sx_i, sy_i), nb))

        if meeting_pt is None:
            return None

        # ── Stitch forward + backward chains ─────────────────────────────────
        fwd = [meeting_pt]
        cur = meeting_pt
        while cur in cf:
            cur = cf[cur]
            fwd.append(cur)
        fwd.reverse()

        bwd = []
        cur = meeting_pt
        while cur in cb:
            cur = cb[cur]
            bwd.append(cur)

        return self._build_path_from_cells(
            fwd + bwd,
            start_pose.header.frame_id,
            info, grid
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Path assembly (shared by both planners)
    # ─────────────────────────────────────────────────────────────────────────

    def _build_path(self, came_from, current, frame_id, info, grid):
        """Reconstruct cell chain from came_from dict, then build Path."""
        cells = [current]
        while current in came_from:
            current = came_from[current]
            cells.append(current)
        cells.reverse()
        return self._build_path_from_cells(cells, frame_id, info, grid)

    def _build_path_from_cells(self, cells, frame_id, info, grid):
        """
        Given a list of grid cells:
          1. Optionally smooth (line-of-sight)
          2. Densify so consecutive waypoints are <= waypoint_spacing metres apart
          3. Assign heading (yaw) to every pose from direction of travel
          4. Return a nav_msgs/Path
        """
        if self.smooth_path:
            cells = self._smooth_cells(cells, grid, info)

        # Convert cells → world (x, y) coords
        world_pts = [self.map_to_world(mx, my, info) for mx, my in cells]

        # ── Densification ─────────────────────────────────────────────────────
        if self.waypoint_spacing > 0.0:
            world_pts = self._densify(world_pts, self.waypoint_spacing)

        # ── Build Path with headings ──────────────────────────────────────────
        path       = Path()
        path.header = Header()
        path.header.stamp    = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        n = len(world_pts)
        for i, (x, y) in enumerate(world_pts):
            # Heading: direction to the next point (or same as previous at end)
            if i < n - 1:
                nx, ny = world_pts[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            elif n >= 2:
                px, py = world_pts[i - 1]
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = 0.0

            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x    = x
            pose.pose.position.y    = y
            pose.pose.position.z    = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        return path

    # ─────────────────────────────────────────────────────────────────────────
    # Smoothing — line-of-sight (Theta* style)
    # ─────────────────────────────────────────────────────────────────────────

    def _smooth_cells(self, cells, grid, info):
        """Remove intermediate cells that have line-of-sight to a farther cell."""
        if len(cells) <= 2:
            return cells

        smoothed    = [cells[0]]
        current_idx = 0

        while current_idx < len(cells) - 1:
            farthest = current_idx + 1
            for test in range(len(cells) - 1, current_idx, -1):
                if self._line_of_sight(cells[current_idx], cells[test], grid, info):
                    farthest = test
                    break
            smoothed.append(cells[farthest])
            current_idx = farthest

        return smoothed

    def _line_of_sight(self, p1, p2, grid, info):
        """Bresenham line-of-sight check."""
        x0, y0 = p1
        x1, y1 = p2
        dx = abs(x1 - x0);  dy = abs(y1 - y0)
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
                err -= dy; x += sx
            if e2 < dx:
                err += dx; y += sy
        return True

    # ─────────────────────────────────────────────────────────────────────────
    # Densification — interpolate between sparse waypoints
    # ─────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _densify(world_pts, spacing):
        """
        Insert interpolated points so no two consecutive waypoints are
        more than `spacing` metres apart.
        """
        if len(world_pts) < 2:
            return world_pts

        dense = [world_pts[0]]
        for i in range(1, len(world_pts)):
            x0, y0 = world_pts[i - 1]
            x1, y1 = world_pts[i]
            seg_len = math.hypot(x1 - x0, y1 - y0)

            if seg_len <= spacing:
                dense.append((x1, y1))
                continue

            # Number of intermediate steps needed
            steps = int(math.ceil(seg_len / spacing))
            for s in range(1, steps + 1):
                t = s / steps
                xi = x0 + t * (x1 - x0)
                yi = y0 + t * (y1 - y0)
                dense.append((xi, yi))

        return dense


# ─────────────────────────────────────────────────────────────────────────────

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