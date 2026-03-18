#!/usr/bin/env python3
"""
actor_safety_node.py  —  v3
============================
Two jobs:
  1. Slow / stop actors when the car is nearby
     → publishes /actors/<n>/pause  and  /actors/<n>/speed_scale
  2. Tell the car to stop when an actor is very close
     → publishes /actors/contact  (String)
     → publishes /actor_safety/car_stop  (Bool)   ← controller reads this

Auto-discovers actors from live /actors/*/pose topics.
Prints clear diagnostics on startup and every 10 s.
"""

import math
import re
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


class ActorSafetyNode(Node):
    def __init__(self):
        super().__init__('actor_safety_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('stop_dist',         1.8)   # actor hard stop + car stop
        self.declare_parameter('warn_dist',         3.5)   # actor start slowing
        self.declare_parameter('clear_dist',        4.2)   # hysteresis — resume above this
        self.declare_parameter('car_slow_dist',     4.0)   # car starts slowing when actor this close
        self.declare_parameter('car_crawl_dist',    1.5)   # car at minimum speed below this
        self.declare_parameter('robot_pose_topic',  '/robot_pose')
        self.declare_parameter('actor_prefix',      'actor_')
        # Manual override — leave blank for auto-discovery
        self.declare_parameter('actor_names',       '')

        self.stop_dist     = float(self.get_parameter('stop_dist').value)
        self.warn_dist     = float(self.get_parameter('warn_dist').value)
        self.clear_dist    = float(self.get_parameter('clear_dist').value)
        self.car_slow_dist  = float(self.get_parameter('car_slow_dist').value)
        self.car_crawl_dist = float(self.get_parameter('car_crawl_dist').value)
        robot_pose_topic   = self.get_parameter('robot_pose_topic').value
        self.actor_prefix  = self.get_parameter('actor_prefix').value

        raw = self.get_parameter('actor_names').value.strip()
        self.manual_names = [n.strip() for n in raw.split(',') if n.strip()] if raw else []

        # ── State ─────────────────────────────────────────────────────
        self.robot_pose      = None
        self.actor_state     = {}   # name → {pose, paused, scale}
        self.subscribed_names = set()

        # Per-actor pub/sub
        self.pause_pubs = {}
        self.scale_pubs = {}
        self.pose_subs  = {}

        # ── Publishers ─────────────────────────────────────────────────
        self.contact_pub  = self.create_publisher(String,      '/actors/contact',            10)
        self.car_speed_pub = self.create_publisher(Float64,    '/actor_safety/car_speed_scale', 10)
        self.marker_pub   = self.create_publisher(MarkerArray, '/actors/proximity_markers',  10)

        # ── Subscribers ────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, robot_pose_topic, self._robot_pose_cb, 10)

        # ── Timers ─────────────────────────────────────────────────────
        self.create_timer(2.0, self._discover_actors)   # find new actors
        self.create_timer(0.1, self._monitor)           # 10 Hz safety check
        self.create_timer(10.0, self._status_log)       # periodic health report

        # Register manual names immediately
        for name in self.manual_names:
            self._register_actor(name)

        self.get_logger().info('=' * 55)
        self.get_logger().info('  Actor Safety Node  v3')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'  actor_stop  : {self.stop_dist} m')
        self.get_logger().info(f'  car_slow    : {self.car_slow_dist} m → {self.car_crawl_dist} m')
        self.get_logger().info(f'  actor_warn  : {self.warn_dist} m')
        self.get_logger().info(f'  actor_prefix: "{self.actor_prefix}"')
        self.get_logger().info(f'  robot_pose  : {robot_pose_topic}')
        if self.manual_names:
            self.get_logger().info(f'  manual names: {self.manual_names}')
        else:
            self.get_logger().info('  mode: auto-discovery (watching /actors/*/pose)')
        self.get_logger().info('=' * 55)

    # ─────────────────────────────────────────────────────────────────
    # Auto-discovery
    # ─────────────────────────────────────────────────────────────────
    def _discover_actors(self):
        pattern = re.compile(r'^/actors/(.+)/pose$')
        for topic_name, _ in self.get_topic_names_and_types():
            m = pattern.match(topic_name)
            if not m:
                continue
            name = m.group(1)
            if not name.startswith(self.actor_prefix):
                continue
            if name not in self.subscribed_names:
                self.get_logger().info(f'  [discovery] Found actor: "{name}"')
                self._register_actor(name)

    def _register_actor(self, name: str):
        if name in self.subscribed_names:
            return
        self.subscribed_names.add(name)
        self.actor_state[name] = {'pose': None, 'paused': False, 'scale': 1.0}

        self.pause_pubs[name] = self.create_publisher(
            Bool,    f'/actors/{name}/pause',       10)
        self.scale_pubs[name] = self.create_publisher(
            Float64, f'/actors/{name}/speed_scale', 10)
        self.pose_subs[name]  = self.create_subscription(
            PoseStamped,
            f'/actors/{name}/pose',
            lambda msg, n=name: self._actor_pose_cb(msg, n),
            10)

        self.get_logger().info(
            f'  Registered "{name}"\n'
            f'    sub  ← /actors/{name}/pose\n'
            f'    pub  → /actors/{name}/pause\n'
            f'    pub  → /actors/{name}/speed_scale'
        )

    # ─────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────
    def _robot_pose_cb(self, msg: PoseStamped):
        self.robot_pose = msg

    def _actor_pose_cb(self, msg: PoseStamped, name: str):
        self.actor_state[name]['pose'] = (
            msg.pose.position.x,
            msg.pose.position.y)

    # ─────────────────────────────────────────────────────────────────
    # Main monitor — 10 Hz
    # ─────────────────────────────────────────────────────────────────
    def _monitor(self):
        if self.robot_pose is None:
            return
        if not self.actor_state:
            return

        rx = self.robot_pose.pose.position.x
        ry = self.robot_pose.pose.position.y

        contacts     = []
        car_scale    = 1.0   # worst (lowest) scale across all actors
        markers      = MarkerArray()

        for idx, (name, state) in enumerate(self.actor_state.items()):
            if state['pose'] is None:
                continue

            ax, ay = state['pose']
            dist   = math.hypot(ax - rx, ay - ry)

            pause_pub = self.pause_pubs[name]
            scale_pub = self.scale_pubs[name]

            # ── Car speed scale ───────────────────────────────────────
            # Ramp from 1.0 (at car_slow_dist) down to 0.1 (at car_crawl_dist).
            # Car never fully stops — actor already stopped/slowed to yield.
            if dist <= self.car_slow_dist:
                span = max(1e-6, self.car_slow_dist - self.car_crawl_dist)
                this_scale = (dist - self.car_crawl_dist) / span
                this_scale = max(0.1, min(1.0, this_scale))
                # Take the MINIMUM across all actors (closest actor wins)
                car_scale = min(car_scale, this_scale)
                if this_scale < 1.0:
                    contacts.append(name)
            # else: this actor is far — don't reset car_scale for other actors

            # ── Actor zone logic ──────────────────────────────────────
            if dist <= self.stop_dist:
                # HARD STOP — freeze actor
                if not state['paused']:
                    state['paused'] = True
                    self.get_logger().warn(
                        f'🛑 STOP: "{name}" {dist:.2f}m from car — frozen')
                # Publish every tick — don't rely on one message getting through
                pause_pub.publish(Bool(data=True))
                scale_pub.publish(Float64(data=0.0))

            elif dist <= self.warn_dist:
                # SLOW — smooth ramp
                if state['paused']:
                    state['paused'] = False
                    pause_pub.publish(Bool(data=False))

                span  = max(1e-6, self.warn_dist - self.stop_dist)
                scale = (dist - self.stop_dist) / span      # 0.0 → 1.0
                scale = max(0.08, min(1.0, scale))

                # Only republish if scale changed significantly (avoids spam)
                if abs(scale - state['scale']) > 0.05:
                    state['scale'] = scale
                    scale_pub.publish(Float64(data=scale))

                self.get_logger().info(
                    f'⚠  "{name}" {dist:.2f}m → scale={scale:.2f}',
                    throttle_duration_sec=2.0)

            elif dist > self.clear_dist:
                # CLEAR — full speed
                if state['paused'] or state['scale'] < 1.0:
                    state['paused'] = False
                    state['scale']  = 1.0
                    pause_pub.publish(Bool(data=False))
                    scale_pub.publish(Float64(data=1.0))
                    self.get_logger().info(f'✓ "{name}" cleared ({dist:.2f}m)')

            # ── Marker ────────────────────────────────────────────────
            m = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'actor_safety'
            m.id                 = idx
            m.type               = Marker.CYLINDER
            m.action             = Marker.ADD
            m.pose.position.x    = ax
            m.pose.position.y    = ay
            m.pose.position.z    = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = self.warn_dist * 2.0
            m.scale.y = self.warn_dist * 2.0
            m.scale.z = 0.05
            m.color.a = 0.20
            if dist <= self.stop_dist:
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0
            elif dist <= self.warn_dist:
                t = (dist - self.stop_dist) / max(1e-6, self.warn_dist - self.stop_dist)
                m.color.r = 1.0; m.color.g = t; m.color.b = 0.0
            else:
                m.color.r = 0.0; m.color.g = 0.8; m.color.b = 0.0
            m.lifetime = Duration(sec=1, nanosec=0)
            markers.markers.append(m)

        # ── Publish car speed scale ───────────────────────────────────
        # Published every tick — DWA reads this to reduce vmax near actors
        self.car_speed_pub.publish(Float64(data=car_scale))

        if contacts:
            c = String(); c.data = ','.join(contacts)
            self.contact_pub.publish(c)

        if markers.markers:
            self.marker_pub.publish(markers)

    # ─────────────────────────────────────────────────────────────────
    # Health check — every 10 s
    # ─────────────────────────────────────────────────────────────────
    def _status_log(self):
        self.get_logger().info(
            f'[safety] robot_pose={"OK" if self.robot_pose else "MISSING ⚠"} '
            f'actors={list(self.actor_state.keys())} '
            f'poses={[n for n,s in self.actor_state.items() if s["pose"] is not None]}'
        )


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ActorSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()