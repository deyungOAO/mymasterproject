#!/usr/bin/env python3
"""
DWA Local Planner — Local Costmap Edition
═══════════════════════════════════════════════════════════════════════

Changes vs previous version:
  + Actor speed scale: subscribes to /actor_safety/car_speed_scale (Float64)
    → ramps vmax down as actor approaches, never hard stops
    → prevents the deadlock where both car and actor wait forever

Collision model
  Subscribes to /local_costmap (rolling grid, robot-centred).
  Each DWA trajectory point → O(1) grid lookup.
  Inflation is baked into the costmap, so DWA steers away early.

Obstacle avoidance pipeline
  1. Actor speed scale  — vmax ramp from actor_safety_node (Float64 0.1–1.0).
  2. Front clearance (raw scan ±45°) — hard emergency stop.
  3. DWA trajectory sampling — costmap lethal-threshold gating.
  4. Waypoint skip — skips blocked wps, lets DWA route around via costmap.
  5. Escape creep — commits to one direction when DWA finds nothing.
  6. Auto-replan — republishes final goal after replan_after_s seconds.
"""

import math
import threading
import collections
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                        DurabilityPolicy, HistoryPolicy)
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

HISTORY_LEN = 300


# ══════════════════════════════════════════════════════════════
#  Dashboard
# ══════════════════════════════════════════════════════════════

class Dashboard:
    FRONT_HALF = math.radians(45)
    SIDE_HALF  = math.radians(135)

    def __init__(self):
        self._lock = threading.Lock()
        self._t0   = time.time()
        mk = lambda: collections.deque(maxlen=HISTORY_LEN)
        (self._t, self._lin, self._ang,
         self._front, self._left, self._right, self._rear,
         self._prog) = [mk() for _ in range(8)]
        threading.Thread(target=self._run, daemon=True).start()

    def push(self, lin, ang, scan, progress_pct):
        f, l, r, b = self._sector_mins(scan)
        with self._lock:
            self._t.append(time.time() - self._t0)
            self._lin.append(lin);   self._ang.append(ang)
            self._front.append(f);   self._left.append(l)
            self._right.append(r);   self._rear.append(b)
            self._prog.append(progress_pct)

    def _sector_mins(self, scan):
        if scan is None:
            return (float('nan'),) * 4
        ang = scan.angle_min
        inc = scan.angle_increment
        rmin, rmax = scan.range_min, scan.range_max
        best = [float('inf')] * 4
        for r in scan.ranges:
            a = ang % (2 * math.pi)
            if a > math.pi:
                a -= 2 * math.pi
            if rmin < r < rmax:
                if   abs(a) <= self.FRONT_HALF:                       idx = 0
                elif a >  self.FRONT_HALF and a <=  self.SIDE_HALF:   idx = 1
                elif a < -self.FRONT_HALF and a >= -self.SIDE_HALF:   idx = 2
                else:                                                   idx = 3
                if r < best[idx]:
                    best[idx] = r
            ang += inc
        return tuple(v if math.isfinite(v) else float('nan') for v in best)

    def _run(self):
        BG, PAN = '#1e1e2e', '#2a2a3e'
        GR, TX  = '#313244', '#cdd6f4'
        C_V, C_W = '#89b4fa', '#a6e3a1'
        C_F, C_L, C_R, C_B = '#f38ba8', '#89b4fa', '#fab387', '#6c7086'
        C_P = '#cba6f7'

        fig, axes = plt.subplots(4, 1, figsize=(10, 10))
        ax_v, ax_w, ax_o, ax_p = axes
        fig.suptitle('DWA Planner — Live Telemetry',
                     fontsize=12, fontweight='bold', color=TX)
        fig.patch.set_facecolor(BG)
        for ax in axes:
            ax.set_facecolor(PAN)
            ax.tick_params(colors=TX)
            ax.yaxis.label.set_color(TX)
            ax.xaxis.label.set_color(TX)
            for sp in ax.spines.values():
                sp.set_edgecolor('#45475a')
            ax.grid(True, color=GR, lw=0.5, ls='--')

        lv,  = ax_v.plot([], [], color=C_V, lw=1.6, label='linear (m/s)')
        lw_, = ax_w.plot([], [], color=C_W, lw=1.6, label='angular (rad/s)')
        lf,  = ax_o.plot([], [], color=C_F, lw=2.0, label='front ±45°')
        ll,  = ax_o.plot([], [], color=C_L, lw=1.3, label='left  45–135°', alpha=0.85)
        lr,  = ax_o.plot([], [], color=C_R, lw=1.3, label='right 45–135°', alpha=0.85)
        lb,  = ax_o.plot([], [], color=C_B, lw=1.0, label='rear >135°',    alpha=0.55)
        ax_o.axhline(y=1.0, color='#f38ba8', ls='--', lw=1, label='safe 1 m')
        lp,  = ax_p.plot([], [], color=C_P, lw=1.6, label='path progress (%)')
        ax_p.set_ylim(-2, 105)

        for ax, lines in [(ax_v, [lv]), (ax_w, [lw_]),
                          (ax_o, [lf, ll, lr, lb]), (ax_p, [lp])]:
            ax.legend(loc='upper left', fontsize=7, facecolor=GR,
                      labelcolor=TX, framealpha=0.8,
                      ncol=2 if ax is ax_o else 1)
        ax_v.set_ylabel('Lin vel (m/s)')
        ax_w.set_ylabel('Ang vel (rad/s)')
        ax_o.set_ylabel('Sector dist (m)')
        ax_p.set_ylabel('Progress (%)')
        ax_p.set_xlabel('Time (s)')

        def mktxt(ax):
            return ax.text(0.99, 0.88, '', transform=ax.transAxes,
                           ha='right', va='top', color='#89dceb',
                           fontsize=9, fontfamily='monospace')
        tv, tw, tf, tp2 = mktxt(ax_v), mktxt(ax_w), mktxt(ax_o), mktxt(ax_p)
        plt.tight_layout(rect=[0, 0, 1, 0.96])

        def _update(_):
            with self._lock:
                if len(self._t) < 2:
                    return
                t   = list(self._t);    lin = list(self._lin)
                ang = list(self._ang);  fv  = list(self._front)
                lv2 = list(self._left); rv  = list(self._right)
                bv  = list(self._rear); pg  = list(self._prog)

            x0, x1 = max(0, t[-1] - 25), t[-1] + 0.5

            def draw(ax, pairs):
                ax.set_xlim(x0, x1)
                all_v = []
                for ln, dat in pairs:
                    ln.set_data(t, dat)
                    all_v += [v for v in dat if math.isfinite(v)]
                if all_v:
                    lo, hi = min(all_v), max(all_v)
                    pad = max(0.3, (hi - lo) * 0.15)
                    ax.set_ylim(lo - pad, hi + pad)

            draw(ax_v, [(lv, lin)])
            draw(ax_w, [(lw_, ang)])
            draw(ax_o, [(lf, fv), (ll, lv2), (lr, rv), (lb, bv)])
            lp.set_data(t, pg)
            ax_p.set_xlim(x0, x1)

            tv.set_text(f'v = {lin[-1]:+.3f} m/s')
            tw.set_text(f'w = {ang[-1]:+.3f} rad/s')
            fd = fv[-1]
            lf.set_color('#f38ba8' if math.isfinite(fd) and fd < 1.0 else C_F)
            tf.set_text(f'front = {"N/A" if not math.isfinite(fd) else f"{fd:.2f} m"}')
            tp2.set_text(f'{pg[-1]:.0f} %')

        self._ani = animation.FuncAnimation(
            fig, _update, interval=80, blit=False, cache_frame_data=False)
        plt.show()


# ══════════════════════════════════════════════════════════════
#  DWA Local Planner Node
# ══════════════════════════════════════════════════════════════

class DWALocalPlanner(Node):

    def __init__(self):
        super().__init__('dwa_local_planner')
        self._declare_parameters()
        self._load_parameters()
        self._setup_pubsub()
        self._init_state()
        self.dash = Dashboard()
        self.create_timer(0.05, self._loop)
        self._log_startup()

    # ── setup ─────────────────────────────────────────────────

    def _declare_parameters(self):
        p = self.declare_parameter
        # Motion
        p('max_speed',              2.0)
        p('min_speed',             -0.3)
        p('max_yaw_rate',           0.8)
        p('max_accel',              1.0)
        p('velocity_samples',       10)
        p('yaw_rate_samples',       15)
        p('predict_time',           1.0)
        p('dt',                     0.1)
        # Cost weights
        p('heading_cost_gain',      2.0)
        p('distance_cost_gain',     1.0)
        p('velocity_cost_gain',     2.0)
        p('obstacle_cost_gain',     1.5)
        p('progress_cost_gain',     3.0)
        # Path tracking
        p('waypoint_tolerance',     0.5)
        p('final_goal_tolerance',   0.4)
        p('lookahead_distance',     2.5)
        # Safety
        p('front_emergency_dist',   1.0)
        p('min_obstacle_distance',  1.2)
        p('safe_obstacle_distance', 1.5)
        p('speed_reduction_factor', 0.5)
        p('robot_radius',           1.5)
        # Obstacle handling
        p('obstacle_wait_time',     0.5)
        p('detour_enabled',         True)
        p('detour_attempt_time',    60.0)
        p('escape_creep_speed',     0.3)
        p('replan_after_s',         8.0)
        # Costmap
        p('costmap_lethal_threshold', 70)
        p('wp_skip_threshold',        90)
        p('wp_skip_max',               2)
        # Topics
        p('robot_pose_topic',    '/robot_pose')
        p('path_topic',          '/planned_path')
        p('cmd_vel_topic',       '/cmd_vel')
        p('scan_topic',          '/scan')
        p('odom_topic',          '/odom')
        p('local_costmap_topic', '/local_costmap')
        p('local_path_topic',    '/dwa_local_path')
        p('goal_topic',          '/goal_pose')
        # Actor safety — published by actor_safety_node.py
        p('car_speed_topic',     '/actor_safety/car_speed_scale')

    def _load_parameters(self):
        g = lambda n: self.get_parameter(n).value
        self.max_speed          = g('max_speed')
        self.min_speed          = g('min_speed')
        self.max_yaw_rate       = g('max_yaw_rate')
        self.max_accel          = g('max_accel')
        self.v_samples          = int(g('velocity_samples'))
        self.w_samples          = int(g('yaw_rate_samples'))
        self.predict_time       = g('predict_time')
        self.dt                 = g('dt')
        self.heading_gain       = g('heading_cost_gain')
        self.distance_gain      = g('distance_cost_gain')
        self.velocity_gain      = g('velocity_cost_gain')
        self.obstacle_gain      = g('obstacle_cost_gain')
        self.progress_gain      = g('progress_cost_gain')
        self.wp_tol             = g('waypoint_tolerance')
        self.final_tol          = g('final_goal_tolerance')
        self.lookahead_dist     = g('lookahead_distance')
        self.front_emerg        = g('front_emergency_dist')
        self.min_obs_dist       = g('min_obstacle_distance')
        self.safe_obs_dist      = g('safe_obstacle_distance')
        self.speed_red          = g('speed_reduction_factor')
        self.robot_radius       = g('robot_radius')
        self.obs_wait_time      = g('obstacle_wait_time')
        self.detour_enabled     = g('detour_enabled')
        self.detour_timeout     = g('detour_attempt_time')
        self.escape_creep_speed = float(g('escape_creep_speed'))
        self.replan_after_s     = float(g('replan_after_s'))
        self.lethal_thresh      = int(g('costmap_lethal_threshold'))
        self.wp_skip_thr        = int(g('wp_skip_threshold'))
        self.wp_skip_max        = int(g('wp_skip_max'))

    def _setup_pubsub(self):
        g = lambda n: self.get_parameter(n).value
        best_effort_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PoseStamped,   g('robot_pose_topic'),    self._pose_cb,      10)
        self.create_subscription(Path,          g('path_topic'),          self._path_cb,      10)
        self.create_subscription(LaserScan,     g('scan_topic'),          self._scan_cb,      10)
        self.create_subscription(Odometry,      g('odom_topic'),          self._odom_cb,      10)
        self.create_subscription(OccupancyGrid, g('local_costmap_topic'), self._costmap_cb,   best_effort_qos)
        # ── Actor safety stop ──────────────────────────────────────────
        # actor_safety_node publishes True when any actor is within car_stop_dist.
        # We subscribe with depth=1 so we always get the latest value, not a stale queue.
        self.create_subscription(Float64,       g('car_speed_topic'),     self._car_speed_cb, 1)

        self.cmd_pub           = self.create_publisher(Twist,       g('cmd_vel_topic'),    10)
        self.local_path_pub    = self.create_publisher(Path,        g('local_path_topic'), 10)
        self.goal_pub          = self.create_publisher(PoseStamped, g('goal_topic'),        10)
        self.target_marker_pub = self.create_publisher(Marker,      '/dwa_target_point',   10)

    def _init_state(self):
        self.pose             = None
        self.scan             = None
        self.cur_vel          = 0.0
        self.cur_yaw_rate     = 0.0
        self.current_path     = None
        self.wp_idx           = 0
        self.total_wps        = 0
        self._final_goal: PoseStamped = None
        self._path_frame: str         = 'map'
        self._cmap_info       = None
        self._cmap_arr        = None
        self.blocked_start    = None
        self.detour_start     = None
        self.is_waiting       = False
        self.is_detouring     = False
        self._escape_dir      = 0.0
        self._last_replan_t   = None
        self._replan_requested = False
        self._hb_tick         = 0
        # Actor speed scale — 1.0=full speed, 0.1=crawl, set by actor_safety_node
        self._actor_speed_scale = 1.0

    def _log_startup(self):
        g = lambda n: self.get_parameter(n).value
        self.get_logger().info('=' * 60)
        self.get_logger().info('DWA Local Planner started')
        self.get_logger().info(f'  lethal threshold  : {self.lethal_thresh}')
        self.get_logger().info(f'  max_speed         : {self.max_speed} m/s')
        self.get_logger().info(f'  robot_radius      : {self.robot_radius} m')
        self.get_logger().info(f'  front_emergency   : {self.front_emerg} m')
        self.get_logger().info(f'  lookahead_distance: {self.lookahead_dist} m')
        self.get_logger().info(f'  local_path_topic  : {g("local_path_topic")}')
        self.get_logger().info(f'  car_speed_topic   : {g("car_speed_topic")}')
        self.get_logger().info('=' * 60)

    # ── callbacks ─────────────────────────────────────────────

    def _pose_cb(self, m: PoseStamped): self.pose = m
    def _scan_cb(self, m: LaserScan):   self.scan = m

    def _odom_cb(self, m: Odometry):
        self.cur_vel      = m.twist.twist.linear.x
        self.cur_yaw_rate = m.twist.twist.angular.z

    def _costmap_cb(self, m: OccupancyGrid):
        self._cmap_info = m.info
        self._cmap_arr  = np.array(m.data, dtype=np.int16).reshape(
            (m.info.height, m.info.width))

    def _car_speed_cb(self, m: Float64):
        """Receive actor proximity speed scale from actor_safety_node.
        1.0 = full speed, 0.1 = crawl near actor, never hard stop.
        """
        prev = self._actor_speed_scale
        self._actor_speed_scale = float(m.data)
        if prev >= 0.99 and self._actor_speed_scale < 0.99:
            self.get_logger().warn(
                f'⚠ Actor nearby — speed capped to {self._actor_speed_scale:.0%}')
        elif prev < 0.99 and self._actor_speed_scale >= 0.99:
            self.get_logger().info('✓ Actor cleared — full speed restored')

    def _path_cb(self, m: Path):
        if not m.poses:
            self.current_path = None
            self.total_wps    = 0
            return

        # Downsample dense A* paths: keep waypoints >= 0.5 m apart.
        poses = m.poses
        if len(poses) > 20:
            kept = [poses[0]]
            for p in poses[1:-1]:
                if math.hypot(p.pose.position.x - kept[-1].pose.position.x,
                              p.pose.position.y - kept[-1].pose.position.y) >= 0.5:
                    kept.append(p)
            kept.append(poses[-1])
            sparse = Path()
            sparse.header = m.header
            sparse.poses  = kept
            m = sparse
            self.get_logger().info(
                f'Path downsampled: {len(poses)} → {len(m.poses)} waypoints')

        self.current_path      = m
        self.wp_idx            = 0
        self.total_wps         = len(m.poses)
        self._final_goal       = m.poses[-1]
        self._path_frame       = m.header.frame_id or 'map'
        self._last_replan_t    = None
        self._replan_requested = False
        self._reset_blocked()
        self._escape_dir = 0.0
        self.get_logger().info(f'✓ Path received: {self.total_wps} waypoints')

    # ── state helpers ─────────────────────────────────────────

    def _reset_blocked(self):
        self.blocked_start = None
        self.detour_start  = None
        self.is_waiting    = False
        self.is_detouring  = False

    def _progress_pct(self):
        return 0.0 if self.total_wps == 0 else 100.0 * self.wp_idx / self.total_wps

    def _stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _yaw(pose) -> float:
        q = pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _norm(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── costmap helpers ───────────────────────────────────────

    def _world_to_cell(self, wx, wy):
        info = self._cmap_info
        if info is None:
            return None
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        return (col, row) if (0 <= col < info.width and 0 <= row < info.height) else None

    def _cell_cost(self, wx, wy) -> int:
        if self._cmap_arr is None:
            return 0
        cr = self._world_to_cell(wx, wy)
        return 0 if cr is None else max(0, int(self._cmap_arr[cr[1], cr[0]]))

    def _is_lethal(self, wx, wy) -> bool:
        return self._cell_cost(wx, wy) >= self.lethal_thresh

    # ── scan helpers ──────────────────────────────────────────

    def _front_clearance(self):
        if self.scan is None:
            return None
        ang = self.scan.angle_min
        inc = self.scan.angle_increment
        rmin, rmax = self.scan.range_min, self.scan.range_max
        best = float('inf')
        for r in self.scan.ranges:
            a = ang % (2 * math.pi)
            if a > math.pi:
                a -= 2 * math.pi
            if abs(a) <= math.radians(45) and rmin < r < rmax:
                best = min(best, r)
            ang += inc
        return best if math.isfinite(best) else None

    def _gap_yaw_rate(self) -> float:
        if self.scan is None:
            return 0.0
        r = np.array(self.scan.ranges, dtype=np.float32)
        a = self.scan.angle_min + np.arange(len(r)) * self.scan.angle_increment
        r = np.where(np.isfinite(r), r, self.scan.range_max)
        N, bs = 36, 2 * math.pi / 36
        means = np.array([
            r[(a >= -math.pi + i * bs) & (a < -math.pi + (i + 1) * bs)].mean()
            if np.any((a >= -math.pi + i * bs) & (a < -math.pi + (i + 1) * bs))
            else self.scan.range_max
            for i in range(N)
        ])
        best_a = -math.pi + (int(np.argmax(means)) + 0.5) * bs
        return float(np.clip(best_a * 1.5, -self.max_yaw_rate, self.max_yaw_rate))

    # ── DWA core ──────────────────────────────────────────────

    def _predict(self, v, w) -> list:
        x, y, yaw = (self.pose.pose.position.x,
                     self.pose.pose.position.y,
                     self._yaw(self.pose.pose))
        traj, t = [(x, y, yaw)], 0.0
        while t < self.predict_time:
            x   += v * math.cos(yaw) * self.dt
            y   += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            traj.append((x, y, yaw))
            t   += self.dt
        return traj

    def _collides(self, traj) -> bool:
        return any(self._is_lethal(x, y) for x, y, _ in traj)

    def _traj_mean_cost(self, traj) -> float:
        if not traj:
            return 0.0
        return sum(self._cell_cost(x, y) for x, y, _ in traj) / len(traj)

    def _cost(self, traj, v, goals) -> float:
        ex, ey, eyaw = traj[-1]
        g0x = goals[0].pose.position.x
        g0y = goals[0].pose.position.y
        heading  = abs(self._norm(math.atan2(g0y - ey, g0x - ex) - eyaw))
        dist     = min(math.hypot(ex - gp.pose.position.x, ey - gp.pose.position.y)
                       for gp in goals)
        vel_cost = (self.max_speed - v) / self.max_speed
        progress = -(math.hypot(self.pose.pose.position.x - g0x,
                                self.pose.pose.position.y - g0y)
                     - math.hypot(ex - g0x, ey - g0y))
        obs_cost = self._traj_mean_cost(traj) / 100.0
        return (self.heading_gain  * heading   +
                self.distance_gain * dist      +
                self.velocity_gain * vel_cost  +
                self.progress_gain * progress  +
                self.obstacle_gain * obs_cost)

    def _dwa(self, vmax, goals):
        if self.pose is None or not goals:
            return None, None, []

        dv   = self.max_accel * self.dt
        v_lo = max(0.0, self.cur_vel - dv)
        v_hi = max(0.4, min(vmax, self.cur_vel + dv))
        w_lo, w_hi = -self.max_yaw_rate, self.max_yaw_rate

        bc, bv, bw, btraj = float('inf'), None, None, []
        for v in np.linspace(v_lo, v_hi, self.v_samples):
            if v < 0.15:
                continue
            for w in np.linspace(w_lo, w_hi, self.w_samples):
                traj = self._predict(v, w)
                if self._collides(traj):
                    continue
                cost = self._cost(traj, v, goals)
                if cost < bc:
                    bc, bv, bw, btraj = cost, v, w, traj

        return bv, bw, btraj

    # ── path helpers ──────────────────────────────────────────

    def _lookahead_goals(self) -> list:
        if self.pose is None or self.current_path is None:
            return []
        poses = self.current_path.poses
        goals, arc = [], 0.0
        prev_x = poses[self.wp_idx].pose.position.x
        prev_y = poses[self.wp_idx].pose.position.y
        for i in range(self.wp_idx, self.total_wps):
            px, py = poses[i].pose.position.x, poses[i].pose.position.y
            if i > self.wp_idx:
                arc += math.hypot(px - prev_x, py - prev_y)
            prev_x, prev_y = px, py
            goals.append(poses[i])
            if arc > self.lookahead_dist:
                break
        return goals



    def _skip_blocked_waypoints(self) -> bool:
        """
        Skip waypoints that are inside a lethal costmap cell.

        No local A* detour — DWA's costmap-based trajectory sampling already
        steers around obstacles.  Injecting detour waypoints behind the robot
        causes u-turns.  We just advance past the blocked wp and let DWA find
        its own path to the next clean one.

        Returns True if any waypoint was skipped (caller should re-check goals).
        Triggers a global replan if we skip past the final waypoint.
        """
        if self._cmap_arr is None or self.current_path is None or self.pose is None:
            return False
        if self.wp_idx >= self.total_wps:
            return False

        rx = self.pose.pose.position.x
        ry = self.pose.pose.position.y
        local_check_dist = max(self.lookahead_dist, 3.0)

        skipped = 0
        while self.wp_idx < self.total_wps and skipped < self.wp_skip_max:
            gp   = self.current_path.poses[self.wp_idx]
            wx, wy = gp.pose.position.x, gp.pose.position.y
            dist = math.hypot(wx - rx, wy - ry)
            cost = self._cell_cost(wx, wy)

            # Only check waypoints that are close enough to be relevant
            if dist > local_check_dist:
                break
            # Keep waypoint if it's not actually blocked
            if cost < self.wp_skip_thr:
                break

            self.get_logger().warn(
                f'⏭ WP {self.wp_idx} blocked (cost={cost} dist={dist:.2f}m) — skipping to next',
                throttle_duration_sec=0.5)
            self.wp_idx += 1
            skipped     += 1

        if skipped == 0:
            return False

        if self.wp_idx >= self.total_wps:
            self.get_logger().warn('⚠ All remaining waypoints blocked — requesting global replan')
            self._try_replan('all waypoints blocked')
            return True

        self.get_logger().info(
            f'⏭ Skipped {skipped} blocked wp(s) — now targeting wp {self.wp_idx}/{self.total_wps}')
        return True


    # ── blocked / escape ──────────────────────────────────────

    def _handle_blocked(self, goals):
        now = self.get_clock().now()

        if self.blocked_start is None:
            self.blocked_start = now
            self.is_waiting    = True
            self.get_logger().warn('⚠  Blocked — waiting...')
            self._stop()
            return

        blocked_s = (now - self.blocked_start).nanoseconds / 1e9
        if blocked_s < self.obs_wait_time:
            self._stop()
            return

        if not self.detour_enabled:
            self.get_logger().error('✗ Blocked; detour disabled')
            self._stop()
            return

        if self.detour_start is None:
            # Before committing to escape rotation, try DWA one more time.
            # If it finds a valid trajectory from the current heading, use it
            # and skip escape entirely (avoids unnecessary spinning).
            if self.current_path and self.pose:
                goals = self._lookahead_goals()
                if goals:
                    vmax = self.max_speed * self._actor_speed_scale
                    bv, bw, btraj = self._dwa(max(0.15, vmax), goals)
                    if bv is not None:
                        self.get_logger().info('✓ DWA found path — skipping escape')
                        self._reset_blocked()
                        cmd = Twist()
                        cmd.linear.x  = bv
                        cmd.angular.z = bw
                        self.cmd_pub.publish(cmd)
                        self._publish_local_path(btraj)
                        return

            self.detour_start = now
            self.is_waiting   = False
            self.is_detouring = True
            self._escape_dir  = self._gap_yaw_rate() or self.max_yaw_rate
            dir_label = 'LEFT' if self._escape_dir > 0 else 'RIGHT'
            self.get_logger().warn(
                f'⚠  Escape started — {dir_label} '
                f'({math.degrees(self._escape_dir):.0f}°/s)')

        detour_s = (now - self.detour_start).nanoseconds / 1e9

        if (self.replan_after_s > 0
                and detour_s > self.replan_after_s
                and not self._replan_requested):
            self._try_replan('stuck in escape')

        if detour_s > self.detour_timeout:
            self.get_logger().error(
                f'✗ Escape timed out after {self.detour_timeout:.0f}s. '
                f'Send a new goal in RViz to replan.',
                throttle_duration_sec=5.0)
            self._stop()
            return

        self._escape_creep()

    def _escape_creep(self):
        """
        Rotate in place only — NO forward motion during escape.

        Driving forward during escape caused the car to head toward
        pedestrians/obstacles in the gap direction, which is unsafe.

        Strategy:
          - Spin in place toward the gap (largest free sector).
          - Every tick, try DWA with the current heading.
          - As soon as DWA finds a valid trajectory, _reset_blocked()
            fires (in _loop) and normal DWA driving resumes.
          - The car never drives forward unless DWA itself decided it
            is safe to do so.
        """
        sign = math.copysign(1.0, self._escape_dir)
        cmd  = Twist()
        cmd.linear.x  = -2.0                          # rotate in place — never drive blind
        self.cmd_pub.publish(cmd)
        time.sleep(1.5)
        cmd.angular.z = sign * self.max_yaw_rate * 0.7
        self.get_logger().info(
            f'  🔄 ESCAPE (rotate-only)  w={cmd.angular.z:+.2f}',
            throttle_duration_sec=0.3)
        self.cmd_pub.publish(cmd)

    def _try_replan(self, reason: str):
        if self._final_goal is None:
            self.get_logger().error(f'✗ Replan ({reason}): no final goal stored')
            return
        now = self.get_clock().now()
        if (self._last_replan_t is not None
                and (now - self._last_replan_t).nanoseconds / 1e9 < self.detour_timeout):
            return
        self._last_replan_t    = now
        self._replan_requested = True
        goal_msg               = PoseStamped()
        goal_msg.header.stamp  = now.to_msg()
        goal_msg.header.frame_id = self._path_frame
        goal_msg.pose          = self._final_goal.pose
        self.goal_pub.publish(goal_msg)
        self.get_logger().warn(
            f'🔄 Auto-replan ({reason}) → '
            f'({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f})')

    # ── main control loop ─────────────────────────────────────

    def _loop(self):
        self.dash.push(self.cur_vel, self.cur_yaw_rate,
                       self.scan, self._progress_pct())

        self._hb_tick += 1
        if self._hb_tick % 100 == 1:
            self._log_state()

        # ── Actor speed scale ──────────────────────────────────────────
        # Applied to vmax below — car slows near actors but never hard stops.
        # This prevents the deadlock where car and actor both wait for each other.

        if self.pose is None or self.current_path is None or self.total_wps == 0:
            return

        # ── Single waypoint advancement (one mechanism only) ──────────
        # Advance wp_idx when within tolerance OR when waypoint is clearly
        # behind the robot (we passed it). Cap at 1 advance per loop to
        # prevent multi-waypoint jumps that send DWA to a distant target.
        poses = self.current_path.poses
        rx, ry = self.pose.pose.position.x, self.pose.pose.position.y
        ryaw   = self._yaw(self.pose.pose)
        hx, hy = math.cos(ryaw), math.sin(ryaw)

        while self.wp_idx < self.total_wps:
            gp   = poses[self.wp_idx]
            gx, gy = gp.pose.position.x, gp.pose.position.y
            dx, dy = gx - rx, gy - ry
            dist   = math.hypot(dx, dy)
            dot    = dx * hx + dy * hy      # >0 = ahead, <0 = behind

            is_final = (self.wp_idx == self.total_wps - 1)
            tol      = self.final_tol if is_final else self.wp_tol

            if dist < tol:
                # Reached — advance
                if is_final:
                    self.get_logger().info('✓ FINAL GOAL REACHED!')
                    self._stop()
                    return
                self.get_logger().info(
                    f'✓ WP {self.wp_idx} → {self.wp_idx + 1}/{self.total_wps}',
                    throttle_duration_sec=0.5)
                self.wp_idx += 1
                self._reset_blocked()
                continue   # re-check new wp immediately (handles clustered wps)

            # Passed without reaching — only skip if genuinely behind AND close
            pass_margin = max(self.wp_tol * 1.5, 1.0)
            if dot < -0.3 and dist < pass_margin:
                self.get_logger().warn(
                    f'[WP_PASSED] idx={self.wp_idx} dist={dist:.2f} dot={dot:.2f}',
                    throttle_duration_sec=0.5)
                self.wp_idx += 1
                continue

            break   # this wp is still ahead — keep targeting it

        if self.wp_idx >= self.total_wps:
            self.get_logger().info('✓ All waypoints reached!',
                                   throttle_duration_sec=5.0)
            self._stop()
            return

        gp   = self.current_path.poses[self.wp_idx]
        dist = math.hypot(self.pose.pose.position.x - gp.pose.position.x,
                          self.pose.pose.position.y - gp.pose.position.y)

        # Publish marker here so it shows on ALL code paths including emergency stop
        self._publish_target_marker()

        # Emergency stop (scan-based)
        fd = self._front_clearance()
        if fd is not None and fd < self.front_emerg:
            if self.cur_vel > 0.15:
                self.get_logger().warn(
                    f'⚠ EMERGENCY STOP front={fd:.2f} m', throttle_duration_sec=0.5)
                self._stop()
            else:
                self._handle_blocked(self._lookahead_goals() or [gp])
            return

        # Speed cap near obstacles
        if fd is not None and fd < self.safe_obs_dist:
            ratio = (fd - self.min_obs_dist) / (self.safe_obs_dist - self.min_obs_dist)
            vmax  = self.max_speed * max(self.speed_red, min(1.0, ratio))
        else:
            vmax = self.max_speed

        # Apply actor proximity speed cap on top of obstacle speed cap
        vmax = vmax * self._actor_speed_scale
        vmax = max(0.15, vmax)   # always keep minimum creep speed

        goals = self._lookahead_goals() or [gp]

        if self._skip_blocked_waypoints():
            return

        bv, bw, btraj = self._dwa(vmax, goals)
        if bv is None:
            self._handle_blocked(goals)
            return

        if abs(bv) > 0.1:
            if self.is_waiting or self.is_detouring:
                self.get_logger().info('✓ Path clear — resuming')
            self._reset_blocked()

        cmd           = Twist()
        cmd.linear.x  = bv
        cmd.angular.z = bw
        self.cmd_pub.publish(cmd)
        self._publish_local_path(btraj)
        self._publish_target_marker()

    # ── diagnostics & visualisation ───────────────────────────

    def _log_state(self):
        pose_ok = self.pose is not None
        path_ok = self.current_path is not None
        cmap_ok = self._cmap_arr is not None
        scan_ok = self.scan is not None
        if not pose_ok or not path_ok:
            self.get_logger().warn(
                f'⏳ DWA waiting  pose={pose_ok}  path={path_ok}  '
                f'costmap={cmap_ok}  scan={scan_ok}')
            return
        if   self.wp_idx >= self.total_wps: state = 'ALL WPS DONE — send new goal'
        elif self._actor_speed_scale < 0.5: state = 'ACTOR SLOW'
        elif self.is_detouring:             state = 'DETOURING'
        elif self.is_waiting:               state = 'WAITING (obstacle)'
        else:                               state = 'RUNNING'
        fd = self._front_clearance()
        self.get_logger().info(
            f'💓 DWA {state} | wp {self.wp_idx}/{self.total_wps} | '
            f'pos ({self.pose.pose.position.x:.2f},'
            f'{self.pose.pose.position.y:.2f}) | '
            f'v={self.cur_vel:.2f} m/s | '
            f'front={f"{fd:.2f} m" if fd else "N/A"} | '
            f'actor_scale={self._actor_speed_scale:.2f} costmap={cmap_ok}')

    def _publish_target_marker(self):
        if self.current_path is None or self.wp_idx >= self.total_wps:
            return
        gp = self.current_path.poses[self.wp_idx]
        m  = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = self.current_path.header.frame_id or 'map'
        m.ns, m.id        = 'dwa_target', 0
        m.type, m.action  = Marker.SPHERE, Marker.ADD
        m.pose.position.x = gp.pose.position.x
        m.pose.position.y = gp.pose.position.y
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        if self._actor_speed_scale < 0.5:
            m.color.r, m.color.g, m.color.b = 1.0, 0.3, 0.0   # orange-red = actor slow
        elif self.is_waiting or self.is_detouring:
            m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0   # orange = blocked
        else:
            m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.3   # green = running
        m.color.a          = 0.9
        m.lifetime.nanosec = int(0.6e9)
        self.target_marker_pub.publish(m)

    def _publish_local_path(self, traj: list):
        if not traj:
            return
        now   = self.get_clock().now().to_msg()
        frame = self.current_path.header.frame_id if self.current_path else 'map'
        path  = Path()
        path.header.stamp    = now
        path.header.frame_id = frame
        for x, y, yaw in traj:
            ps = PoseStamped()
            ps.header.stamp    = now
            ps.header.frame_id = frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        self.local_path_pub.publish(path)


# ── entry point ───────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DWALocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()