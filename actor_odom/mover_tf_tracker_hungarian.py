#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

NAMES = ["vehicle_blue","actor_walking1","actor_walking2","actor_walking3","actor_walking4"]

EXPECTED_XY = {
    "vehicle_blue"  : ( 0.0,  0.0),
    "actor_walking1": ( 3.0, -1.0),
    "actor_walking2": ( 0.0, -5.0),
    "actor_walking3": (-10.0,-3.0),
    "actor_walking4": (-2.0,  0.0),
}

def quat_from_yaw(yaw: float) -> Quaternion:
    s = math.sin(0.5*yaw); c = math.cos(0.5*yaw)
    q = Quaternion(); q.x = 0.0; q.y = 0.0; q.z = s; q.w = c
    return q

def qmul(a: Quaternion, b: Quaternion) -> Quaternion:
    r = Quaternion()
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    return r

def hungarian(cost):  # minimal-cost assignment
    m, n = len(cost), len(cost[0])
    u = [0]*(m+1); v = [0]*(n+1); p = [0]*(n+1); way = [0]*(n+1)
    INF = 10**9
    for i in range(1, m+1):
        p[0] = i
        minv = [INF]*(n+1); used = [False]*(n+1); j0 = 0
        while True:
            used[j0] = True
            i0 = p[j0]; delta = INF; j1 = 0
            for j in range(1, n+1):
                if used[j]: continue
                cur = cost[i0-1][j-1]-u[i0]-v[j]
                if cur < minv[j]: minv[j] = cur; way[j] = j0
                if minv[j] < delta: delta = minv[j]; j1 = j
            for j in range(n+1):
                if used[j]: u[p[j]] += delta; v[j] -= delta
                else: minv[j] -= delta
            j0 = j1
            if p[j0] == 0: break
        while True:
            j1 = way[j0]; p[j0] = p[j1]; j0 = j1
            if j0 == 0: break
    out = [-1]*m
    for j in range(1, n+1):
        if p[j] > 0 and p[j]-1 < m: out[p[j]-1] = j-1
    return out

class MoverTFTracker(Node):
    def __init__(self):
        super().__init__("mover_tf_tracker_publish")
        # params
        self.parent  = self.declare_parameter("parent_frame", "world").get_parameter_value().string_value
        self.in_topic= self.declare_parameter("in_topic", "/world/campus_world/dynamic_pose/info").get_parameter_value().string_value
        self.use_vel_heading = bool(self.declare_parameter("use_velocity_heading", True).get_parameter_value().bool_value)
        self.heading_alpha   = float(self.declare_parameter("heading_smoothing_alpha", 0.4).get_parameter_value().double_value)
        self.actor_yaw_off   = math.radians(float(self.declare_parameter("actor_yaw_offset_deg", -90.0).get_parameter_value().double_value))
        self.vehicle_yaw_off = math.radians(float(self.declare_parameter("vehicle_yaw_offset_deg", 0.0).get_parameter_value().double_value))
        self.max_match_dist  = float(self.declare_parameter("max_match_dist", 8.0).get_parameter_value().double_value)
        self.publish_every_n = int(self.declare_parameter("publish_every_n", 1).get_parameter_value().integer_value)
        self.debug           = bool(self.declare_parameter("debug", False).get_parameter_value().bool_value)

        # height filters to ignore skeleton joints near origin
        self.car_z_min,   self.car_z_max   = float(self.declare_parameter("car_z_min",   -0.4).get_parameter_value().double_value), float(self.declare_parameter("car_z_max",   0.4).get_parameter_value().double_value)
        self.actor_z_min, self.actor_z_max = float(self.declare_parameter("actor_z_min",  0.3).get_parameter_value().double_value), float(self.declare_parameter("actor_z_max",  2.0).get_parameter_value().double_value)

        # io
        self.br  = TransformBroadcaster(self)
        self.sub = self.create_subscription(PoseArray, self.in_topic, self.cb, 10)
        self.ps_pub = {n: self.create_publisher(PoseStamped, f"/tracked/{n}/pose", 10) for n in NAMES}

        # state
        self.last_xy = {n: EXPECTED_XY.get(n, (0.0,0.0)) for n in NAMES}
        self.yaw_lp  = {n: 0.0 for n in NAMES}
        self.bootstrapped = False
        self.counter = 0

        self.get_logger().info(f"Listening: {self.in_topic}, parent='{self.parent}'")

    def cb(self, pa: PoseArray):
        poses = pa.poses
        if not poses:
            return
        self.counter += 1
        if self.counter % self.publish_every_n:
            return

        obs = [(p.position.x, p.position.y, p.position.z, p.orientation) for p in poses]

        # split by height
        car_idxs   = [i for i,(x,y,z,q) in enumerate(obs) if self.car_z_min   <= z <= self.car_z_max]
        actor_idxs = [i for i,(x,y,z,q) in enumerate(obs) if self.actor_z_min <= z <= self.actor_z_max]

        # bootstrap
        if not self.bootstrapped:
            if car_idxs:
                veh_i = min(car_idxs, key=lambda i: math.hypot(obs[i][0]-self.last_xy["vehicle_blue"][0],
                                                               obs[i][1]-self.last_xy["vehicle_blue"][1]))
            else:
                veh_i = min(range(len(obs)), key=lambda i: obs[i][2])
            self.last_xy["vehicle_blue"] = (obs[veh_i][0], obs[veh_i][1])
            for name in NAMES:
                if name == "vehicle_blue": continue
                ex, ey = EXPECTED_XY[name]
                best = min(range(len(obs)), key=lambda i: (obs[i][0]-ex)**2+(obs[i][1]-ey)**2)
                self.last_xy[name] = (obs[best][0], obs[best][1])
            self.bootstrapped = True
            if self.debug: self.get_logger().info("Bootstrapped with height filters.")

        # vehicle: nearest car candidate to last_xy (fallback min-Z)
        if car_idxs:
            vlx, vly = self.last_xy["vehicle_blue"]
            veh_i = min(car_idxs, key=lambda i: math.hypot(obs[i][0]-vlx, obs[i][1]-vly))
        else:
            veh_i = min(range(len(obs)), key=lambda i: obs[i][2])

        # actors: Hungarian among actor candidates
        actor_names = [n for n in NAMES if n != "vehicle_blue"]
        BIG = 1e6
        name_to_idx = {"vehicle_blue": veh_i}

        if actor_idxs:
            cost = []
            for name in actor_names:
                lx, ly = self.last_xy[name]
                row = []
                for i in actor_idxs:
                    x,y,_,_ = obs[i]
                    d = math.hypot(x-lx, y-ly)
                    row.append(d if d <= self.max_match_dist else BIG)
                cost.append(row)
            cols = hungarian(cost)
            for r, c in enumerate(cols):
                if c is None or c < 0 or c >= len(actor_idxs): continue
                if cost[r][c] >= BIG: continue
                name_to_idx[actor_names[r]] = actor_idxs[c]

        # greedy fallback for any unassigned actor
        remaining_names = [n for n in actor_names if n not in name_to_idx]
        remaining_idxs  = [i for i in actor_idxs if i not in name_to_idx.values()]
        for name in remaining_names:
            if not remaining_idxs: break
            lx, ly = self.last_xy[name]
            best = min(remaining_idxs, key=lambda i: math.hypot(obs[i][0]-lx, obs[i][1]-ly))
            name_to_idx[name] = best
            remaining_idxs.remove(best)

        # publish TF + PoseStamped and update state
        stamp = pa.header.stamp
        for name, idx in name_to_idx.items():
            x, y, z, q_base = obs[idx]

            if self.use_vel_heading:
                lx, ly = self.last_xy[name]
                dx, dy = (x - lx), (y - ly)
                if abs(dx)+abs(dy) > 1e-6:
                    yaw = math.atan2(dy, dx)
                    yaw = self.heading_alpha*yaw + (1.0 - self.heading_alpha)*self.yaw_lp[name]
                    self.yaw_lp[name] = yaw
                else:
                    yaw = self.yaw_lp[name]
                q_use = quat_from_yaw(yaw)
            else:
                q_use = q_base

            off = self.actor_yaw_off if name.startswith("actor_") else self.vehicle_yaw_off
            q_out = qmul(q_use, quat_from_yaw(off))

            # TF
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.parent
            t.child_frame_id = name
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation = q_out
            self.br.sendTransform(t)

            # PoseStamped
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.parent
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation = q_out
            self.ps_pub[name].publish(ps)

            self.last_xy[name] = (x, y)

def main():
    rclpy.init()
    node = MoverTFTracker()
    # set to False if you are NOT bridging a clock
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
