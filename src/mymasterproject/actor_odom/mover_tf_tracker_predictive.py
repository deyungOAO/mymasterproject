#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from rclpy.duration import Duration
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
        super().__init__("mover_tf_tracker_predictive")
        # topics / frames
        self.parent  = self.declare_parameter("parent_frame", "world").get_parameter_value().string_value
        self.in_topic= self.declare_parameter("in_topic", "/world/campus_world/dynamic_pose/info").get_parameter_value().string_value

        # association & dynamics
        self.max_match_dist   = float(self.declare_parameter("max_match_dist", 8.0).get_parameter_value().double_value)   # meters (hard gate)
        self.max_speed_mps    = float(self.declare_parameter("max_speed_mps", 5.0).get_parameter_value().double_value)    # for prediction gate
        self.alpha_dist       = float(self.declare_parameter("alpha_dist", 1.0).get_parameter_value().double_value)       # weight for distance
        self.beta_z           = float(self.declare_parameter("beta_z",   0.2).get_parameter_value().double_value)         # weight for z penalty
        self.actor_z_ref      = float(self.declare_parameter("actor_z_ref", 1.0).get_parameter_value().double_value)      # typical actor root height
        self.car_z_ref        = float(self.declare_parameter("car_z_ref",   0.0).get_parameter_value().double_value)      # typical car height
        self.publish_every_n  = int(self.declare_parameter("publish_every_n", 1).get_parameter_value().integer_value)

        # orientation
        self.use_vel_heading  = bool(self.declare_parameter("use_velocity_heading", True).get_parameter_value().bool_value)
        self.heading_alpha    = float(self.declare_parameter("heading_smoothing_alpha", 0.4).get_parameter_value().double_value)
        self.actor_yaw_off    = math.radians(float(self.declare_parameter("actor_yaw_offset_deg", -90.0).get_parameter_value().double_value))
        self.vehicle_yaw_off  = math.radians(float(self.declare_parameter("vehicle_yaw_offset_deg", 0.0).get_parameter_value().double_value))

        # I/O
        self.br  = TransformBroadcaster(self)
        self.sub = self.create_subscription(PoseArray, self.in_topic, self.cb, 10)
        self.ps_pub = {n: self.create_publisher(PoseStamped, f"/tracked/{n}/pose", 10) for n in NAMES}

        # state
        self.last_xy  = {n: EXPECTED_XY.get(n, (0.0,0.0)) for n in NAMES}
        self.last_vel = {n: (0.0, 0.0) for n in NAMES}   # m/s (for prediction)
        self.yaw_lp   = {n: 0.0 for n in NAMES}
        self.last_stamp = None
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

        # collect obs
        obs = [(p.position.x, p.position.y, p.position.z, p.orientation) for p in poses]

        # dt for prediction (seconds)
        dt = 0.0
        if self.last_stamp is not None:
            now = rclpy.time.Time.from_msg(pa.header.stamp)
            dt = max(0.0, (now - self.last_stamp).nanoseconds / 1e9)
            self.last_stamp = now
        else:
            self.last_stamp = rclpy.time.Time.from_msg(pa.header.stamp)

        # bootstrap: place tracks near expected XY (car by min |z - car_z_ref|)
        if not self.bootstrapped:
            veh_i = min(range(len(obs)), key=lambda i: abs(obs[i][2] - self.car_z_ref))
            self.last_xy["vehicle_blue"] = (obs[veh_i][0], obs[veh_i][1])
            for name in NAMES:
                if name == "vehicle_blue": continue
                ex, ey = EXPECTED_XY[name]
                best = min(range(len(obs)), key=lambda i: (obs[i][0]-ex)**2+(obs[i][1]-ey)**2 + 0.25*(obs[i][2]-self.actor_z_ref)**2)
                self.last_xy[name] = (obs[best][0], obs[best][1])
            self.bootstrapped = True

        # build cost matrices (track rows x obs cols)
        BIG = 1e6
        costs = []
        for name in NAMES:
            # constant-velocity prediction
            px, py = self.last_xy[name]
            vx, vy = self.last_vel[name]
            pred_x, pred_y = (px + vx*dt, py + vy*dt) if dt > 0.0 else (px, py)
            # dynamic gate: if an obs requires > max_speed_mps, mark as impossible
            row = []
            for (x,y,z,q) in obs:
                dist = math.hypot(x - pred_x, y - pred_y)
                # predicted max move this frame
                max_step = self.max_speed_mps * max(dt, 1e-3)
                if dist > max(self.max_match_dist, max_step*3.0):
                    row.append(BIG)
                else:
                    z_ref = self.actor_z_ref if name.startswith("actor_") else self.car_z_ref
                    z_pen = abs(z - z_ref)
                    row.append(self.alpha_dist*dist + self.beta_z*z_pen)
            costs.append(row)

        # assign with Hungarian
        assign = hungarian(costs)  # row i -> column j

        # publish TF + PoseStamped, update state
        stamp = pa.header.stamp
        for i, name in enumerate(NAMES):
            j = assign[i]
            if j is None or j < 0 or j >= len(obs) or costs[i][j] >= BIG:
                continue
            x, y, z, q_base = obs[j]
            # velocity update (for next-step prediction)
            px, py = self.last_xy[name]
            dt_eff = max(dt, 1e-3)
            vx, vy = ((x - px) / dt_eff, (y - py) / dt_eff)
            # clamp to reasonable speed to avoid spikes
            sp = math.hypot(vx, vy)
            if sp > self.max_speed_mps:
                scale = self.max_speed_mps / (sp + 1e-6)
                vx *= scale; vy *= scale
            self.last_vel[name] = (0.6*self.last_vel[name][0] + 0.4*vx,
                                   0.6*self.last_vel[name][1] + 0.4*vy)

            # orientation
            if self.use_vel_heading:
                yaw = math.atan2(self.last_vel[name][1], self.last_vel[name][0]) if sp > 1e-3 else self.yaw_lp[name]
                yaw = self.heading_alpha*yaw + (1.0 - self.heading_alpha)*self.yaw_lp[name]
                self.yaw_lp[name] = yaw
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

            # state
            self.last_xy[name] = (x, y)

def main():
    rclpy.init()
    node = MoverTFTracker()
    # set True only if your /clock is ticking
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
