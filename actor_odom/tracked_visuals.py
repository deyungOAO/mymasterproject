#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

TRACKED = ["vehicle_blue","actor_walking1","actor_walking2","actor_walking3","actor_walking4"]

class TrackedVisuals(Node):
    def __init__(self):
        super().__init__("tracked_visuals")
        self.parent = self.declare_parameter("parent_frame", "world").get_parameter_value().string_value
        self.trail_len = int(self.declare_parameter("trail_length", 300).get_parameter_value().integer_value)
        self.sphere_d = float(self.declare_parameter("sphere_diameter", 0.35).get_parameter_value().double_value)
        self.arrow_len = float(self.declare_parameter("arrow_length", 0.6).get_parameter_value().double_value)
        self.arrow_d   = float(self.declare_parameter("arrow_diameter", 0.06).get_parameter_value().double_value)
        self.rate_hz   = float(self.declare_parameter("publish_rate_hz", 15.0).get_parameter_value().double_value)

        # latest poses + trails
        self.latest = {n: None for n in TRACKED}
        self.trails = {n: Path() for n in TRACKED}
        for n in TRACKED:
            self.trails[n].header.frame_id = self.parent

        # subs & pubs
        for n in TRACKED:
            self.create_subscription(PoseStamped, f"/tracked/{n}/pose",
                                     lambda msg, name=n: self.cb_pose(msg, name), 10)
        self.mark_pub = self.create_publisher(MarkerArray, "/viz/tracked/marker_array", 10)

        self.path_pub = {n: self.create_publisher(Path, f"/viz/tracked/{n}/path", 10) for n in TRACKED}

        self.timer = self.create_timer(1.0/max(self.rate_hz, 1.0), self.tick)
        self.get_logger().info("Tracked visuals running (markers on /viz/tracked/markers, paths on /viz/tracked/<name>/path)")

    def cb_pose(self, msg: PoseStamped, name: str):
        self.latest[name] = msg
        # append to trail
        path = self.trails[name]
        path.header.stamp = msg.header.stamp
        path.poses.append(msg)
        if len(path.poses) > self.trail_len:
            del path.poses[0]

    def tick(self):
        # publish markers + paths
        ma = MarkerArray()
        # clear once at first publish (safe to do every tick)
        clear = Marker(); clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for idx, name in enumerate(TRACKED):
            msg = self.latest[name]
            if msg is None:
                continue

            # choose a stable id base per entity
            base_id = idx * 100

            # sphere
            m = Marker()
            m.header.frame_id = self.parent
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "tracked_sphere"
            m.id = base_id + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = msg.pose
            m.scale.x = m.scale.y = m.scale.z = self.sphere_d
            # simple palette
            colors = [
                (0.20, 0.60, 1.00),  # vehicle blue
                (0.90, 0.30, 0.30),
                (0.30, 0.85, 0.40),
                (0.95, 0.75, 0.20),
                (0.70, 0.40, 0.95),
            ]
            r,g,b = colors[idx % len(colors)]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            ma.markers.append(m)

            # heading arrow
            a = Marker()
            a.header = m.header
            a.ns = "tracked_arrow"
            a.id = base_id + 2
            a.type = Marker.ARROW
            a.action = Marker.ADD
            a.pose = msg.pose
            a.scale.x = self.arrow_len     # length
            a.scale.y = self.arrow_d       # shaft diameter
            a.scale.z = self.arrow_d * 1.8 # head diameter
            a.color.r, a.color.g, a.color.b, a.color.a = r, g, b, 0.9
            ma.markers.append(a)

            # label
            t = Marker()
            t.header = m.header
            t.ns = "tracked_label"
            t.id = base_id + 3
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose = msg.pose
            t.pose.position.z += self.sphere_d * 0.9
            t.scale.z = 0.3
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
            t.text = name
            ma.markers.append(t)

            # publish path
            self.path_pub[name].publish(self.trails[name])

        self.mark_pub.publish(ma)

def main():
    rclpy.init()
    node = TrackedVisuals()
    # flip this to True only if you’re bridging /clock and using sim time
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
