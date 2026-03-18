#!/usr/bin/env python3
"""
Dynamic Global Costmap
══════════════════════════════════════════════════════════════════════
Replaces the static-only version.

Two-layer architecture:
  Layer 1 (static)  — computed ONCE when the /map arrives.
                      Full distance-transform inflation of all
                      walls/obstacles in the YAML map.
                      Stored in self._base_cost  (float32 H×W).

  Layer 2 (dynamic) — updated every scan callback.
                      Scan rays are projected into map frame using
                      the latest robot pose, marked as occupied, then
                      inflated with a fast pre-built circular brush.
                      Stored in self._dyn_cost   (float32 H×W).

Published costmap = max(layer1, layer2), clipped to 0-100.
Published at up to `update_hz` Hz (default 5 Hz) so the global
planner always sees dynamic obstacles in time to replan.

Topics:
  Sub: /map              (nav_msgs/OccupancyGrid, latched)
  Sub: /scan             (sensor_msgs/LaserScan)
  Sub: /robot_pose       (geometry_msgs/PoseStamped)
  Pub: /global_costmap/costmap  (nav_msgs/OccupancyGrid, latched)

Parameters:
  inflation_radius        (float, m)    static inflation from wall surface
  robot_radius            (float, m)    inscribed radius, full cost inside
  cost_scaling_factor     (float)       k in 100·exp(-k·d) decay
  dynamic_inflation_r     (float, m)    inflation around scan hits (def 1.0)
  dynamic_cost_peak       (float 0-100) cost at centre of scan hit (def 90)
  occupied_threshold      (int)         map cell value → static obstacle
  update_hz               (float)       max publish rate (default 5.0)
  scan_decay_s            (float)       seconds before dynamic layer fades
  scan_range_max_override (float)       ignore rays beyond this (0=use sensor)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy,
                        ReliabilityPolicy, HistoryPolicy)
import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


class DynamicGlobalCostmap(Node):

    def __init__(self):
        super().__init__('dynamic_global_costmap')

        # ── parameters ─────────────────────────────────────────
        self.declare_parameter('inflation_radius',        5.0)
        self.declare_parameter('robot_radius',            1.5)
        self.declare_parameter('cost_scaling_factor',     2.0)
        self.declare_parameter('dynamic_inflation_r',     0.3)
        self.declare_parameter('dynamic_cost_peak',      90.0)
        self.declare_parameter('occupied_threshold',      65)
        self.declare_parameter('update_hz',               5.0)
        self.declare_parameter('scan_decay_s',            2.0)
        self.declare_parameter('scan_range_max_override', 0.0)
        # Only insert scan hits closer than this (filters far wall returns)
        self.declare_parameter('scan_insert_max_m',   4.0)
        # Sub-sample: use every Nth scan ray (reduces coverage + CPU)
        self.declare_parameter('scan_step',            5)
        self.declare_parameter('input_map_topic',    '/map')
        self.declare_parameter('scan_topic',         '/scan')
        self.declare_parameter('robot_pose_topic',   '/robot_pose')
        self.declare_parameter('output_costmap_topic',
                               '/global_costmap/costmap')

        def g(n): return self.get_parameter(n).value

        self.inflation_radius    = float(g('inflation_radius'))
        self.robot_radius        = float(g('robot_radius'))
        self.cost_scaling_factor = float(g('cost_scaling_factor'))
        self.dyn_inflation_r     = float(g('dynamic_inflation_r'))
        self.dyn_cost_peak       = float(g('dynamic_cost_peak'))
        self.occ_thresh          = int(g('occupied_threshold'))
        self.update_hz           = float(g('update_hz'))
        self.scan_decay_s        = float(g('scan_decay_s'))
        self.range_max_override  = float(g('scan_range_max_override'))
        self.scan_insert_max_m   = float(g('scan_insert_max_m'))
        self.scan_step           = max(1, int(g('scan_step')))

        # ── state ──────────────────────────────────────────────
        self._map_info    = None     # MapMetaData
        self._base_cost   = None     # float32 H×W — static layer
        self._dyn_cost    = None     # float32 H×W — dynamic layer
        self._dyn_brush   = None     # pre-built inflation brush (float32)
        self._dyn_r_cells = 0
        self._last_scan_t = 0.0
        self._robot_pose  = None
        self._dirty       = False

        # ── QoS ────────────────────────────────────────────────
        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── subscriptions ──────────────────────────────────────
        self.create_subscription(
            OccupancyGrid, g('input_map_topic'),
            self._map_cb, latched_qos)
        self.create_subscription(
            LaserScan, g('scan_topic'),
            self._scan_cb, sensor_qos)
        self.create_subscription(
            PoseStamped, g('robot_pose_topic'),
            self._pose_cb, 10)

        # ── publisher ──────────────────────────────────────────
        self._pub = self.create_publisher(
            OccupancyGrid, g('output_costmap_topic'), latched_qos)

        # ── publish timer ──────────────────────────────────────
        self.create_timer(1.0 / self.update_hz, self._publish_cb)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Dynamic Global Costmap')
        self.get_logger().info(f'  Static inflation  : {self.inflation_radius} m')
        self.get_logger().info(f'  Robot radius      : {self.robot_radius} m')
        self.get_logger().info(f'  Dynamic inflation : {self.dyn_inflation_r} m  '
                               f'peak={self.dyn_cost_peak}')
        self.get_logger().info(f'  Publish rate      : {self.update_hz} Hz')
        self.get_logger().info(f'  Scan decay        : {self.scan_decay_s} s')
        self.get_logger().info('  Waiting for /map ...')
        self.get_logger().info('=' * 60)

    # ── callbacks ─────────────────────────────────────────────

    def _pose_cb(self, m: PoseStamped):
        self._robot_pose = m

    def _map_cb(self, msg: OccupancyGrid):
        """Receive static map → compute base cost layer (once)."""
        if self._base_cost is not None:
            return   # map is static, ignore duplicates

        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height} '
            f'@ {msg.info.resolution} m/cell')

        self._map_info = msg.info
        res = msg.info.resolution
        W, H = msg.info.width, msg.info.height

        data      = np.array(msg.data, dtype=np.int16).reshape((H, W))
        obstacles = (data >= self.occ_thresh).astype(np.uint8)
        free      = (obstacles == 0).astype(np.uint8)

        dist_px   = cv2.distanceTransform(free, cv2.DIST_L2, 5)
        dist_m    = dist_px * res

        # Static cost layer
        base = np.zeros((H, W), dtype=np.float32)
        base[obstacles == 1] = 100.0
        mask     = (obstacles == 0) & (dist_m <= self.inflation_radius)
        dist_rel = np.maximum(dist_m[mask] - self.robot_radius, 0.0)
        base[mask] = np.clip(
            100.0 * np.exp(-self.cost_scaling_factor * dist_rel), 0.0, 100.0)

        self._base_cost = base
        self._dyn_cost  = np.zeros((H, W), dtype=np.float32)

        # Pre-build circular inflation brush for scan hits
        r      = max(1, int(self.dyn_inflation_r / res))
        self._dyn_r_cells = r
        ys, xs = np.mgrid[-r:r+1, -r:r+1]
        d_m    = np.sqrt(xs**2 + ys**2) * res
        brush  = np.clip(
            self.dyn_cost_peak * np.exp(
                -self.cost_scaling_factor * np.maximum(d_m - self.robot_radius, 0.0)),
            0.0, self.dyn_cost_peak).astype(np.float32)
        brush[d_m > self.dyn_inflation_r] = 0.0
        self._dyn_brush = brush

        self.get_logger().info(
            f'  Static layer OK — obstacles={int(obstacles.sum())}  '
            f'inflated={int(mask.sum())}')
        self.get_logger().info(
            f'  Dynamic brush: {2*r+1}x{2*r+1} cells '
            f'({self.dyn_inflation_r} m radius)')
        self._dirty = True

    def _scan_cb(self, msg: LaserScan):
        """Project scan hits into map frame → update dynamic layer.

        Only marks hits that are:
          1. Within scan_insert_max_m (ignores far-away/max-range wall returns)
          2. Not at sensor max range (those are usually noise, not real objects)
          3. Sub-sampled every scan_step rays (reduces CPU + coverage)
        """
        if self._base_cost is None or self._robot_pose is None:
            return

        info = self._map_info
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y
        W, H = info.width, info.height

        rx   = self._robot_pose.pose.position.x
        ry   = self._robot_pose.pose.position.y
        q    = self._robot_pose.pose.orientation
        ryaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                          1 - 2*(q.y*q.y + q.z*q.z))
        cos_r, sin_r = math.cos(ryaw), math.sin(ryaw)

        # Only use sensor max if no override; subtract small margin so
        # rays hitting exactly at max range are filtered (usually noise)
        rmax_sensor = (self.range_max_override
                       if self.range_max_override > 0.0
                       else msg.range_max)
        rmax_insert = min(rmax_sensor * 0.98, self.scan_insert_max_m)
        rmin = msg.range_min
        r    = self._dyn_r_cells
        brush= self._dyn_brush
        step = self.scan_step   # sub-sample: only every Nth ray

        dyn  = np.zeros_like(self._dyn_cost)

        for i in range(0, len(msg.ranges), step):
            rng = msg.ranges[i]
            if not (rmin < rng < rmax_insert):
                continue

            ang = msg.angle_min + i * msg.angle_increment
            lx  = rng * math.cos(ang)
            ly  = rng * math.sin(ang)
            wx  = rx + lx * cos_r - ly * sin_r
            wy  = ry + lx * sin_r + ly * cos_r

            col = int((wx - ox) / res)
            row = int((wy - oy) / res)
            if col < 0 or row < 0 or col >= W or row >= H:
                continue

            r0c = max(0, row - r);  r1c = min(H, row + r + 1)
            c0c = max(0, col - r);  c1c = min(W, col + r + 1)
            r0b = r0c - (row - r);  r1b = r0b + (r1c - r0c)
            c0b = c0c - (col - r);  c1b = c0b + (c1c - c0c)
            np.maximum(dyn[r0c:r1c, c0c:c1c],
                       brush[r0b:r1b, c0b:c1b],
                       out=dyn[r0c:r1c, c0c:c1c])

        self._dyn_cost    = dyn
        self._last_scan_t = time.time()
        self._dirty       = True

    def _publish_cb(self):
        """Merge static + dynamic layers and publish."""
        if self._base_cost is None or not self._dirty:
            return
        self._dirty = False

        # Fade dynamic layer if scan is stale
        age = time.time() - self._last_scan_t
        if self._last_scan_t > 0 and age > self.scan_decay_s:
            fade = max(0.0, 1.0 - (age - self.scan_decay_s) / self.scan_decay_s)
            dyn  = self._dyn_cost * fade
        else:
            dyn  = self._dyn_cost

        merged    = np.clip(np.maximum(self._base_cost, dyn), 0.0, 100.0)
        cost_int8 = merged.astype(np.int8)

        out = OccupancyGrid()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.info            = self._map_info
        out.data            = cost_int8.flatten().tolist()
        self._pub.publish(out)

        dyn_cells = int((dyn > 10).sum())
        self.get_logger().info(
            f'Costmap published — dynamic cells: {dyn_cells}  '
            f'scan age: {age:.1f}s',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicGlobalCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()