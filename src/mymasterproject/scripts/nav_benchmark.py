#!/usr/bin/env python3
"""
nav_benchmark_multi.py - Multi-Run Navigation Benchmark
========================================================
Runs N navigation attempts automatically.
After each run, resets the robot to spawn position via Gazebo service,
waits for it to settle, then publishes the goal again.

Usage:
    python3 nav_benchmark_multi.py --ros-args \\
        -p use_sim_time:=true \\
        -p goal_x:=16.0 \\
        -p goal_y:=0.0 \\
        -p num_runs:=10 \\
        -p spawn_x:=-20.0 \\
        -p spawn_y:=0.0

Results saved to nav_benchmark_results.csv (appended each run).
"""

import math
import csv
import time
import os
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


class NavBenchmarkMulti(Node):

    STUCK_DIST_M  = 0.05
    STUCK_TIME_S  = 15.0

    def __init__(self):
        super().__init__('nav_benchmark_multi')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('goal_x',             16.0)
        self.declare_parameter('goal_y',              0.0)
        self.declare_parameter('goal_frame',         'map')
        self.declare_parameter('goal_topic',         '/goal_pose')
        self.declare_parameter('robot_pose_topic',   '/robot_pose')
        self.declare_parameter('path_topic',         '/planned_path')
        self.declare_parameter('goal_tolerance',      0.8)
        self.declare_parameter('timeout_s',         240.0)
        self.declare_parameter('num_runs',           10)
        self.declare_parameter('spawn_x',           -20.0)
        self.declare_parameter('spawn_y',             0.0)
        self.declare_parameter('spawn_z',             0.5)
        self.declare_parameter('model_name',         'ackermann_car')
        self.declare_parameter('reset_wait_s',        5.0)   # wait after reset before publishing goal
        self.declare_parameter('first_run_delay_s',   5.0)   # wait before very first run
        self.declare_parameter('output_csv',         'nav_benchmark_results.csv')

        self.goal_x          = float(self.get_parameter('goal_x').value)
        self.goal_y          = float(self.get_parameter('goal_y').value)
        self.goal_frame      = self.get_parameter('goal_frame').value
        self.goal_topic      = self.get_parameter('goal_topic').value
        self.robot_pose_topic= self.get_parameter('robot_pose_topic').value
        self.path_topic      = self.get_parameter('path_topic').value
        self.tolerance       = float(self.get_parameter('goal_tolerance').value)
        self.timeout_s       = float(self.get_parameter('timeout_s').value)
        self.num_runs        = int(self.get_parameter('num_runs').value)
        self.spawn_x         = float(self.get_parameter('spawn_x').value)
        self.spawn_y         = float(self.get_parameter('spawn_y').value)
        self.spawn_z         = float(self.get_parameter('spawn_z').value)
        self.model_name      = self.get_parameter('model_name').value
        self.reset_wait_s    = float(self.get_parameter('reset_wait_s').value)
        self.first_delay_s   = float(self.get_parameter('first_run_delay_s').value)
        self.output_csv      = self.get_parameter('output_csv').value

        # ── Publishers & Subscribers ──────────────────────────────────
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.create_subscription(PoseStamped, self.robot_pose_topic, self._pose_cb, 10)

        # ── State ─────────────────────────────────────────────────────
        self.robot_pose       = None
        self.prev_pos         = None
        self.path_len_m       = 0.0
        self.run_idx          = 0
        self.run_active       = False
        self.run_start_time   = 0.0
        self.start_robot_pos  = (0.0, 0.0)
        self.last_move_pos    = (0.0, 0.0)
        self.last_move_time   = 0.0
        self.waiting_for_reset= False

        # ── CSV ───────────────────────────────────────────────────────
        self._init_csv()

        # ── Timers ────────────────────────────────────────────────────
        self._watchdog_timer = self.create_timer(0.1, self._watchdog)
        # Start first run after delay
        self._start_timer = self.create_timer(self.first_delay_s, self._start_next_run)

        # ── Banner ────────────────────────────────────────────────────
        self.get_logger().info(f"{BOLD}{CYAN}{'='*55}{RESET}")
        self.get_logger().info(f"{BOLD}{CYAN}  Multi-Run Navigation Benchmark{RESET}")
        self.get_logger().info(f"{BOLD}{CYAN}{'='*55}{RESET}")
        self.get_logger().info(f"  Goal     : ({self.goal_x}, {self.goal_y})")
        self.get_logger().info(f"  Runs     : {self.num_runs}")
        self.get_logger().info(f"  Spawn    : ({self.spawn_x}, {self.spawn_y})")
        self.get_logger().info(f"  Tolerance: {self.tolerance} m")
        self.get_logger().info(f"  Timeout  : {self.timeout_s} s")
        self.get_logger().info(f"  CSV      : {os.path.abspath(self.output_csv)}")
        self.get_logger().info(f"  Starting in {self.first_delay_s:.0f}s ...")
        self.get_logger().info(f"{BOLD}{CYAN}{'='*55}{RESET}")

    # ──────────────────────────────────────────────────────────────────
    # Run management
    # ──────────────────────────────────────────────────────────────────
    def _start_next_run(self):
        """Called once per run to reset robot and publish goal."""
        # Cancel the start timer (one-shot per run)
        if hasattr(self, '_start_timer') and self._start_timer:
            self._start_timer.cancel()

        if self.run_idx >= self.num_runs:
            self._all_done()
            return

        self.run_idx += 1
        self.get_logger().info(
            f"\n{BOLD}{YELLOW}{'='*55}{RESET}"
            f"\n{BOLD}{YELLOW}  RUN {self.run_idx} / {self.num_runs}{RESET}"
            f"\n{BOLD}{YELLOW}{'='*55}{RESET}"
        )

        # Reset robot to spawn position
        self._reset_robot()

        # Wait for robot to settle, then publish goal
        self.waiting_for_reset = True
        self._reset_wait_timer = self.create_timer(self.reset_wait_s, self._after_reset)

    def _after_reset(self):
        self._reset_wait_timer.cancel()
        self.waiting_for_reset = False

        # Init run state
        self.path_len_m    = 0.0
        self.prev_pos      = None
        self.run_active    = True
        self.run_start_time = time.time()

        rx = self.robot_pose.pose.position.x if self.robot_pose else self.spawn_x
        ry = self.robot_pose.pose.position.y if self.robot_pose else self.spawn_y
        self.start_robot_pos  = (rx, ry)
        self.last_move_pos    = (rx, ry)
        self.last_move_time   = self.run_start_time

        # Publish goal
        msg = PoseStamped()
        msg.header.frame_id    = self.goal_frame
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.pose.position.x    = self.goal_x
        msg.pose.position.y    = self.goal_y
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

        dist = math.hypot(self.goal_x - rx, self.goal_y - ry)
        self.get_logger().info(f"  Goal published. Robot at ({rx:.2f}, {ry:.2f}), dist={dist:.2f}m")

    def _reset_robot(self):
        """Teleport robot back to spawn using gz service."""
        self.get_logger().info(f"  Resetting robot to ({self.spawn_x}, {self.spawn_y}) ...")
        try:
            cmd = [
                'gz', 'service',
                '-s', f'/world/campus_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req',
                f'name: "{self.model_name}" '
                f'position: {{x: {self.spawn_x}, y: {self.spawn_y}, z: {self.spawn_z}}} '
                f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if result.returncode == 0:
                self.get_logger().info("  ✓ Robot reset via gz service")
            else:
                self.get_logger().warn(f"  gz service failed: {result.stderr[:80]}")
                self._reset_robot_fallback()
        except Exception as e:
            self.get_logger().warn(f"  gz service error: {e} — trying fallback")
            self._reset_robot_fallback()

    def _reset_robot_fallback(self):
        """Fallback: use ros2 service if gz service not available."""
        try:
            cmd = [
                'ros2', 'service', 'call',
                '/world/campus_world/set_pose',
                'ros_gz_interfaces/srv/SetEntityPose',
                f'{{entity: {{name: "{self.model_name}"}}, '
                f'pose: {{position: {{x: {self.spawn_x}, y: {self.spawn_y}, z: {self.spawn_z}}}, '
                f'orientation: {{w: 1.0}}}}}}'
            ]
            subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            self.get_logger().info("  ✓ Robot reset via ros2 service (fallback)")
        except Exception as e:
            self.get_logger().error(f"  Both reset methods failed: {e}")

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────
    def _pose_cb(self, msg: PoseStamped):
        if self.prev_pos and self.run_active:
            dx   = msg.pose.position.x - self.prev_pos[0]
            dy   = msg.pose.position.y - self.prev_pos[1]
            step = math.hypot(dx, dy)
            if step < 2.0:
                self.path_len_m += step

            cur   = (msg.pose.position.x, msg.pose.position.y)
            moved = math.hypot(cur[0] - self.last_move_pos[0],
                               cur[1] - self.last_move_pos[1])
            if moved > self.STUCK_DIST_M:
                self.last_move_pos  = cur
                self.last_move_time = time.time()

        self.robot_pose = msg
        self.prev_pos   = (msg.pose.position.x, msg.pose.position.y)

    # ──────────────────────────────────────────────────────────────────
    # Watchdog
    # ──────────────────────────────────────────────────────────────────
    def _watchdog(self):
        if not self.run_active or self.robot_pose is None or self.waiting_for_reset:
            return

        now     = time.time()
        elapsed = now - self.run_start_time

        rx   = self.robot_pose.pose.position.x
        ry   = self.robot_pose.pose.position.y
        dist = math.hypot(self.goal_x - rx, self.goal_y - ry)

        # SUCCESS
        if dist <= self.tolerance:
            self._finish_run('SUCCESS', elapsed,
                             f'Reached goal in {elapsed:.1f}s, traveled {self.path_len_m:.1f}m')
            return

        # TIMEOUT
        if elapsed >= self.timeout_s:
            self._finish_run('TIMEOUT', elapsed,
                             f'Timed out, {dist:.2f}m from goal')
            return

        # STUCK
        stuck_for = now - self.last_move_time
        if stuck_for >= self.STUCK_TIME_S and elapsed > 5.0:
            self._finish_run('FAILED', elapsed,
                             f'Stuck {stuck_for:.1f}s, {dist:.2f}m from goal')
            return

        # Progress
        self.get_logger().info(
            f"  [Run {self.run_idx}/{self.num_runs}] "
            f"{elapsed:6.1f}s | dist={dist:.2f}m | traveled={self.path_len_m:.1f}m",
            throttle_duration_sec=10.0
        )

    # ──────────────────────────────────────────────────────────────────
    # Finish one run
    # ──────────────────────────────────────────────────────────────────
    def _finish_run(self, result: str, duration: float, notes: str = ''):
        self.run_active = False
        colour = GREEN if result == 'SUCCESS' else RED

        self.get_logger().info(
            f"\n{BOLD}{colour}  Run {self.run_idx}: {result} — {notes}{RESET}"
        )

        self._csv_writer.writerow({
            'timestamp':     datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'run':           self.run_idx,
            'goal_x':        self.goal_x,
            'goal_y':        self.goal_y,
            'start_x':       round(self.start_robot_pos[0], 3),
            'start_y':       round(self.start_robot_pos[1], 3),
            'duration_s':    round(duration, 2),
            'path_length_m': round(self.path_len_m, 2),
            'result':        result,
            'notes':         notes,
        })
        self._csv_file.flush()

        if self.run_idx >= self.num_runs:
            # Small delay then wrap up
            self.create_timer(2.0, self._all_done)
        else:
            # Start next run after a short pause
            self._start_timer = self.create_timer(3.0, self._start_next_run)

    # ──────────────────────────────────────────────────────────────────
    # All runs complete
    # ──────────────────────────────────────────────────────────────────
    def _all_done(self):
        self.get_logger().info(f"\n{BOLD}{CYAN}{'='*55}{RESET}")
        self.get_logger().info(f"{BOLD}{CYAN}  ALL {self.num_runs} RUNS COMPLETE{RESET}")
        self.get_logger().info(f"{BOLD}{CYAN}{'='*55}{RESET}")
        self._print_summary()
        self.get_logger().info(f"  Results: {os.path.abspath(self.output_csv)}")
        self.get_logger().info(f"{BOLD}{CYAN}{'='*55}{RESET}\n")
        self.create_timer(2.0, lambda: rclpy.shutdown())

    def _print_summary(self):
        try:
            self._csv_file.flush()
            with open(self.output_csv, newline='') as f:
                rows = [r for r in csv.DictReader(f)
                        if int(r.get('run', 0)) <= self.num_runs]
        except Exception:
            return

        if not rows:
            return

        total     = len(rows)
        successes = [r for r in rows if r['result'] == 'SUCCESS']
        rate      = len(successes) / total * 100
        times     = [float(r['duration_s']) for r in successes]
        avg_t     = sum(times) / len(times) if times else 0.0
        best_t    = min(times) if times else 0.0
        worst_t   = max(times) if times else 0.0

        self.get_logger().info(f"  Total runs    : {total}")
        self.get_logger().info(f"  Successes     : {len(successes)}")
        self.get_logger().info(f"  Success rate  : {rate:.1f}%")
        if times:
            self.get_logger().info(f"  Time avg      : {avg_t:.2f}s")
            self.get_logger().info(f"  Time best     : {best_t:.2f}s")
            self.get_logger().info(f"  Time worst    : {worst_t:.2f}s")

    # ──────────────────────────────────────────────────────────────────
    # CSV
    # ──────────────────────────────────────────────────────────────────
    def _init_csv(self):
        fields = ['timestamp', 'run', 'goal_x', 'goal_y',
                  'start_x', 'start_y', 'duration_s',
                  'path_length_m', 'result', 'notes']
        write_header = not os.path.exists(self.output_csv)
        self._csv_file   = open(self.output_csv, 'a', newline='')
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=fields)
        if write_header:
            self._csv_writer.writeheader()
            self._csv_file.flush()

    def destroy_node(self):
        if hasattr(self, '_csv_file'):
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavBenchmarkMulti()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()