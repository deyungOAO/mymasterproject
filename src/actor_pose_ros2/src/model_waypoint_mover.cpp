// model_waypoint_mover.cpp  —  v3  (speed-scale fix)
//
// Fix over previous v3:
//   - startSim stall line inside pause branch removed (was dead code —
//     startSim is not used after init, scaledTime drives everything)
//   - speed_scale=0.0 now correctly freezes actor in place every frame
//   - pause=true also resets scaledTime advancement (correct)
//   - Logs topic names on startup so you can verify in terminal

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <atomic>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace gz;
using namespace gz::sim;

// ─────────────────────────────────────────────────────────────────────────────
struct Keyframe
{
  double           t;
  gz::math::Pose3d pose;
  double           pause{0.0};
};

static double smoothstep(double u)
{ return 0.5 - 0.5 * std::cos(u * M_PI); }

static double wrapAngle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ─────────────────────────────────────────────────────────────────────────────
class ModelWaypointMover
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  void Configure(const Entity &_id,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    this->modelEntity = _id;
    sdf::ElementPtr root = _sdf->Clone();

    this->loop          = root->Get<bool>  ("loop",            true ).first;
    this->autoStart     = root->Get<bool>  ("auto_start",      true ).first;
    this->yawOffsetDeg  = root->Get<double>("yaw_offset_deg",  0.0  ).first;
    this->speed_mps     = root->Get<double>("speed",           1.4  ).first;
    this->faceDirection = root->Get<bool>  ("face_direction",  true ).first;
    this->startDelay    = root->Get<double>("start_delay",     0.0  ).first;
    this->useSmoothing  = root->Get<bool>  ("smooth",          true ).first;

    // ── Waypoints ─────────────────────────────────────────────────────
    std::vector<Keyframe> rawKeys;
    if (root->HasElement("trajectory"))
    {
      sdf::ElementPtr tr = root->GetElement("trajectory");
      for (sdf::ElementPtr wp = tr->GetElement("waypoint"); wp;
           wp = wp->GetNextElement("waypoint"))
      {
        gz::math::Pose3d p{0,0,0,0,0,0};
        if (wp->HasElement("pose"))
          p = wp->GetElement("pose")->Get<gz::math::Pose3d>();

        double t = -1.0;
        if (wp->HasElement("time"))
          t = wp->GetElement("time")->Get<double>();

        double pause = 0.0;
        if (wp->HasElement("pause"))
          pause = wp->GetElement("pause")->Get<double>();

        rawKeys.push_back(Keyframe{t, p, pause});
      }
    }
    if (rawKeys.empty()) return;

    // ── Build timeline ────────────────────────────────────────────────
    bool hasExplicitTime = false;
    for (auto &k : rawKeys)
      if (k.t >= 0.0) { hasExplicitTime = true; break; }

    if (hasExplicitTime)
    {
      std::sort(rawKeys.begin(), rawKeys.end(),
                [](const Keyframe&a, const Keyframe&b){ return a.t < b.t; });
      this->keys = rawKeys;
    }
    else
    {
      // Speed-based: first waypoint starts after startDelay
      this->keys.push_back(Keyframe{this->startDelay, rawKeys[0].pose, rawKeys[0].pause});
      for (size_t i = 1; i < rawKeys.size(); ++i)
      {
        const auto &prev = rawKeys[i-1];
        const auto &curr = rawKeys[i];
        double dist = std::sqrt(
          std::pow(curr.pose.Pos().X() - prev.pose.Pos().X(), 2) +
          std::pow(curr.pose.Pos().Y() - prev.pose.Pos().Y(), 2) +
          std::pow(curr.pose.Pos().Z() - prev.pose.Pos().Z(), 2));
        double seg     = (this->speed_mps > 0.0) ? dist / this->speed_mps : 1.0;
        double prevEnd = this->keys.back().t + this->keys.back().pause;
        this->keys.push_back(Keyframe{prevEnd + seg, curr.pose, curr.pause});
      }
    }

    // ── Auto-yaw ──────────────────────────────────────────────────────
    if (this->faceDirection)
    {
      for (size_t i = 0; i < this->keys.size(); ++i)
      {
        size_t next = (i + 1) % this->keys.size();
        if (!this->loop && i + 1 >= this->keys.size()) break;
        double dx = this->keys[next].pose.Pos().X() - this->keys[i].pose.Pos().X();
        double dy = this->keys[next].pose.Pos().Y() - this->keys[i].pose.Pos().Y();
        if (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6)
        {
          double yaw = std::atan2(dy, dx);
          auto rpy = this->keys[i].pose.Rot().Euler();
          this->keys[i].pose.Set(
            this->keys[i].pose.Pos(),
            gz::math::Quaterniond(rpy.X(), rpy.Y(), yaw));
        }
      }
    }

    this->duration  = this->keys.back().t + this->keys.back().pause;
    this->started   = !this->autoStart;

    // ── ROS ───────────────────────────────────────────────────────────
    std::string modelName = "actor";
    if (auto *nc = _ecm.Component<components::Name>(_id))
      modelName = nc->Data();
    this->myName = modelName;

    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    this->rosNode = std::make_shared<rclcpp::Node>("mover_" + modelName);

    const std::string pauseTopic = "/actors/" + modelName + "/pause";
    const std::string scaleTopic = "/actors/" + modelName + "/speed_scale";

    this->pauseSub = this->rosNode->create_subscription<std_msgs::msg::Bool>(
      pauseTopic, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      { this->externalPause.store(msg->data); });

    this->scaleSub = this->rosNode->create_subscription<std_msgs::msg::Float64>(
      scaleTopic, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg)
      { this->speedScale.store(std::clamp(msg->data, 0.0, 1.0)); });

    // Print clearly so user can verify topics in terminal
    RCLCPP_INFO(this->rosNode->get_logger(),
      "=================================================\n"
      "  model_waypoint_mover ready: %s\n"
      "  Listening for pause:       %s\n"
      "  Listening for speed_scale: %s\n"
      "  Waypoints: %zu  Duration: %.1fs  Speed: %.1fm/s\n"
      "=================================================",
      modelName.c_str(),
      pauseTopic.c_str(),
      scaleTopic.c_str(),
      this->keys.size(),
      this->duration,
      this->speed_mps);
  }

  // ────────────────────────────────────────────────────────────────────
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (this->keys.size() < 2) return;

    rclcpp::spin_some(this->rosNode);

    // Explicit namespace avoids >> template-argument parse ambiguity
    const double dt_s = std::chrono::duration<double>(_info.dt).count();

    // Capture initial sim time on first tick
    if (!this->started)
    {
      this->started = true;
      // scaledTime already 0.0 — correct start
    }

    // ── PAUSE: freeze pose, do NOT advance scaledTime ─────────────────
    if (this->externalPause.load())
    {
      if (this->hasFrozenPose)
        _setCmd(_ecm, this->frozenPose);
      return;   // scaledTime not advanced → actor resumes exactly where it stopped
    }

    // ── SPEED SCALE: advance scaledTime at reduced rate ───────────────
    // scale=1.0 → normal speed
    // scale=0.5 → half speed (takes twice as long between waypoints)
    // scale=0.0 → actor frozen (scaledTime stops)
    const double scale = this->speedScale.load();
    this->scaledTime  += dt_s * scale;

    double t = this->scaledTime;
    if (this->loop && this->duration > 0.0)
      t = std::fmod(this->scaledTime, this->duration);
    else if (t > this->duration)
      t = this->duration;

    // ── Start delay: hold at first waypoint ───────────────────────────
    if (t < this->keys.front().t)
    {
      const gz::math::Pose3d &fp = this->keys.front().pose;
      this->frozenPose    = fp;
      this->hasFrozenPose = true;
      _setCmd(_ecm, fp);
      return;
    }

    // ── Find segment ──────────────────────────────────────────────────
    size_t i = 0;
    for (size_t k = 0; k + 1 < this->keys.size(); ++k)
    {
      if (t < this->keys[k + 1].t) { i = k; break; }
      i = k;
    }

    const Keyframe &A = this->keys[i];
    const Keyframe &B = this->keys[std::min(i + 1, this->keys.size() - 1)];

    // ── Waypoint dwell pause ──────────────────────────────────────────
    if (t < A.t + A.pause)
    {
      this->frozenPose    = A.pose;
      this->hasFrozenPose = true;
      _setCmd(_ecm, A.pose);
      return;
    }

    // ── Interpolate A → B ─────────────────────────────────────────────
    const double tStart = A.t + A.pause;
    const double span   = std::max(1e-6, B.t - tStart);
    double u = std::clamp((t - tStart) / span, 0.0, 1.0);
    if (this->useSmoothing) u = smoothstep(u);

    const gz::math::Vector3d pos =
      A.pose.Pos() * (1.0 - u) + B.pose.Pos() * u;

    double yaw = wrapAngle(
      A.pose.Rot().Yaw() + u * wrapAngle(B.pose.Rot().Yaw() - A.pose.Rot().Yaw()));
    yaw += this->yawOffsetDeg * M_PI / 180.0;

    const gz::math::Pose3d P(pos.X(), pos.Y(), pos.Z(), 0.0, 0.0, yaw);
    this->frozenPose    = P;
    this->hasFrozenPose = true;
    _setCmd(_ecm, P);
  }

private:
  void _setCmd(EntityComponentManager &_ecm, const gz::math::Pose3d &P)
  {
    if (auto *cmd = _ecm.Component<components::WorldPoseCmd>(this->modelEntity))
      cmd->Data() = P;
    else
      _ecm.CreateComponent(this->modelEntity, components::WorldPoseCmd(P));
  }

  Entity                modelEntity{kNullEntity};
  std::vector<Keyframe> keys;
  std::string           myName;

  bool   loop          {true};
  bool   autoStart     {true};
  bool   started       {false};
  bool   faceDirection {true};
  bool   useSmoothing  {true};
  double duration      {0.0};
  double yawOffsetDeg  {0.0};
  double speed_mps     {1.4};
  double startDelay    {0.0};

  // Internal clock — scaled by speedScale each tick
  double scaledTime    {0.0};

  // Last known pose (used when paused or in start-delay)
  gz::math::Pose3d frozenPose;
  bool             hasFrozenPose{false};

  // Thread-safe: written by ROS subscriber thread, read by PreUpdate (sim thread)
  std::atomic<bool>   externalPause{false};
  std::atomic<double> speedScale{1.0};

  rclcpp::Node::SharedPtr                                 rosNode;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    pauseSub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr scaleSub;
};

// ─────────────────────────────────────────────────────────────────────────────
GZ_ADD_PLUGIN(ModelWaypointMover,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ModelWaypointMover, "model_waypoint_mover")