#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

def d2(a, b):  # squared 2D distance
    return (a[0]-b[0])**2 + (a[1]-b[1])**2

class ActorsOnly(Node):
    def __init__(self):
        super().__init__("actors_only_from_posearray")
        # Params
        self.in_topic  = self.declare_parameter("in_topic", "/world/campus_world/dynamic_pose/info").get_parameter_value().string_value
        self.k         = int(self.declare_parameter("k", 5).get_parameter_value().integer_value)  # how many actors to publish
        self.parent    = self.declare_parameter("parent_frame", "world").get_parameter_value().string_value
        self.actor_z_min = float(self.declare_parameter("actor_z_min", 0.2).get_parameter_value().double_value)
        self.actor_z_max = float(self.declare_parameter("actor_z_max", 2.5).get_parameter_value().double_value)
        self.max_match_dist = float(self.declare_parameter("max_match_dist", 8.0).get_parameter_value().double_value)  # meters
        self.publish_markers = bool(self.declare_parameter("publish_markers", True).get_parameter_value().bool_value)
        self.exclude_min_x = bool(self.declare_parameter("exclude_min_x", True).get_parameter_value().bool_value)

        # I/O
        self.sub = self.create_subscription(PoseArray, self.in_topic, self.cb, 10)
        self.pub_pa = self.create_publisher(PoseArray, "/actors/poses", 10)
        self.pub_ps = [self.create_publisher(PoseStamped, f"/actors/actor_{i+1}/pose", 10) for i in range(self.k)]
        self.pub_mk = self.create_publisher(MarkerArray, "/viz/actors/marker_array", 10) if self.publish_markers else None

        # Track state
        self.last_xy = [None] * self.k

        self.get_logger().info(f"Actors-only node: listening {self.in_topic} -> /actors/* (excluding min-X each frame)")

    def cb(self, msg: PoseArray):
        if not msg.poses:
            return

        # 1) Build observations: (pose, x, y, z)
        obs = [(p, p.position.x, p.position.y, p.position.z) for p in msg.poses]

        # 2) Exclude the min-X observation as the vehicle (per your rule)
        if self.exclude_min_x:
            min_x_idx = min(range(len(obs)), key=lambda i: obs[i][1])
            obs = [o for i, o in enumerate(obs) if i != min_x_idx]

        # 3) Keep likely actor roots by Z (wide-ish band; adjust if needed)
        actors = [o for o in obs if self.actor_z_min <= o[3] <= self.actor_z_max]
        if not actors:
            return

        # 4) Greedy nearest-neighbor assignment to keep tracks stable
        used = [False] * len(actors)
        chosen = [-1] * self.k

        # Re-assign existing tracks first
        for t in range(self.k):
            if self.last_xy[t] is None:
                continue
            best_i, best_d2 = -1, 1e18
            for i, (_, x, y, _) in enumerate(actors):
                if used[i]: continue
                dd = d2(self.last_xy[t], (x, y))
                if dd < best_d2:
                    best_d2, best_i = dd, i
            if best_i >= 0 and math.sqrt(best_d2) <= self.max_match_dist:
                chosen[t] = best_i
                used[best_i] = True

        # Initialize any remaining tracks
        for t in range(self.k):
            if chosen[t] != -1: continue
            best_i = next((i for i, u in enumerate(actors) if not u and not used[i]), None)
            # Pick the farthest-from-already-chosen for spread, else just first free
            if best_i is None:
                # nothing left
                continue
            # simple spread heuristic
            best_i, best_score = -1, -1e18
            for i, (_, x, y, _) in enumerate(actors):
                if used[i]: continue
                score = 0.0
                for tt in range(self.k):
                    if chosen[tt] != -1:
                        q = actors[chosen[tt]][0].position
                        score -= d2((x, y), (q.x, q.y))
                if score > best_score:
                    best_score, best_i = score, i
            if best_i is not None and best_i >= 0:
                chosen[t] = best_i
                used[best_i] = True

        # 5) Publish PoseArray + PoseStamped + Markers
        out_pa = PoseArray()
        out_pa.header = msg.header
        out_pa.header.frame_id = self.parent

        markers = MarkerArray()
        if self.pub_mk:
            clear = Marker(); clear.action = Marker.DELETEALL
            markers.markers.append(clear)

        palette = [
            (0.90, 0.30, 0.30),
            (0.30, 0.85, 0.40),
            (0.95, 0.75, 0.20),
            (0.70, 0.40, 0.95),
            (0.20, 0.60, 1.00),
        ]

        for t in range(self.k):
            idx = chosen[t]
            if idx == -1: continue
            p = actors[idx][0]

            # PoseArray aggregate
            out_pa.poses.append(p)

            # PoseStamped per actor_i
            ps = PoseStamped()
            ps.header = msg.header
            ps.header.frame_id = self.parent
            ps.pose = p
            self.pub_ps[t].publish(ps)

            # remember last XY
            self.last_xy[t] = (p.position.x, p.position.y)

            # Optional markers
            if self.pub_mk:
                r, g, b = palette[t % len(palette)]
                base = 100 * (t + 1)
                # sphere
                m = Marker()
                m.header = ps.header
                m.ns = "actors_sphere"; m.id = base + 1
                m.type = Marker.SPHERE; m.action = Marker.ADD
                m.pose = p
                m.scale.x = m.scale.y = m.scale.z = 0.35
                m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
                markers.markers.append(m)
                # label
                lbl = Marker()
                lbl.header = ps.header
                lbl.ns = "actors_label"; lbl.id = base + 2
                lbl.type = Marker.TEXT_VIEW_FACING; lbl.action = Marker.ADD
                lbl.pose = p; lbl.pose.position.z += 0.45
                lbl.scale.z = 0.28
                lbl.color.r = lbl.color.g = lbl.color.b = 1.0; lbl.color.a = 1.0
                lbl.text = f"actor_{t+1}"
                markers.markers.append(lbl)

        self.pub_pa.publish(out_pa)
        if self.pub_mk:
            self.pub_mk.publish(markers)

def main():
    rclpy.init()
    node = ActorsOnly()
    # Set True only if /clock is bridged and ticking
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
