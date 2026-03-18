// model_waypoint_mover.cpp (fixed)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>                         // UpdateInfo
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>               // Pose component
#include <gz/sim/components/PoseCmd.hh>            // WorldPoseCmd lives here
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Element.hh>

using namespace gz;
using namespace gz::sim;

struct Keyframe
{
  double t;                 // seconds
  gz::math::Pose3d pose;    // x y z R P Y
};

class ModelWaypointMover
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  void Configure(const Entity &_id,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &,
                 EventManager &) override
  {
    this->modelEntity = _id;

    // Clone the SDF so we can call non-const APIs like GetElement()
    sdf::ElementPtr root = _sdf->Clone();

    this->loop       = root->Get<bool>("loop", true).first;
    this->autoStart  = root->Get<bool>("auto_start", true).first;
    this->yawOffsetDeg = root->Get<double>("yaw_offset_deg", 0.0).first;

    // Read <trajectory><waypoint>{ <time>, <pose> }...
    if (root->HasElement("trajectory"))
    {
      sdf::ElementPtr tr = root->GetElement("trajectory");
      // First waypoint
      for (sdf::ElementPtr wp = tr->GetElement("waypoint"); wp;
           wp = wp->GetNextElement("waypoint"))
      {
        double t = 0.0;
        if (wp->HasElement("time"))
          t = wp->GetElement("time")->Get<double>();

        gz::math::Pose3d p{0,0,0,0,0,0};
        if (wp->HasElement("pose"))
        {
          // <pose> child element parses directly to Pose3d
          p = wp->GetElement("pose")->Get<gz::math::Pose3d>();
        }
        else
        {
          // Fallback: individual fields (optional)
          auto getd = [&](const char *k, double def=0.0)
          { return wp->HasElement(k) ? wp->GetElement(k)->Get<double>() : def; };
          p = gz::math::Pose3d(
                getd("x"), getd("y"), getd("z"),
                getd("roll"), getd("pitch"), getd("yaw"));
        }
        this->keys.push_back(Keyframe{t, p});
      }
    }

    std::sort(this->keys.begin(), this->keys.end(),
              [](const Keyframe&a, const Keyframe&b){ return a.t < b.t; });
    if (!this->keys.empty())
      this->duration = this->keys.back().t;

    // If autoStart==true, we’ll set startSim on first PreUpdate
    this->started = !this->autoStart;
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (this->keys.size() < 2)
      return;

    using namespace std::chrono;

    if (!this->started)
    {
      this->startSim = _info.simTime;  // simTime is a duration in sim8
      this->started = true;
    }

    // elapsed seconds since start
    const auto elapsed = _info.simTime - this->startSim;
    const double now_s =
    std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();

    double t = now_s;
    if (this->loop && this->duration > 0.0)
      t = std::fmod(now_s, this->duration);
    else if (t > this->duration)
      t = this->duration;

    // find segment [i, i+1] with keys[i].t <= t <= keys[i+1].t
    size_t i = 0;
    while (i + 1 < this->keys.size() && this->keys[i + 1].t < t) ++i;
    const size_t j = std::min(i + 1, this->keys.size() - 1);

    const Keyframe &A = this->keys[i];
    const Keyframe &B = this->keys[j];

    const double span = std::max(1e-6, B.t - A.t);
    const double u = std::clamp((t - A.t) / span, 0.0, 1.0);

    // Lerp position
    const gz::math::Vector3d pos =
      A.pose.Pos() * (1.0 - u) + B.pose.Pos() * u;

    // Lerp yaw with wrap handling
    auto wrap = [](double a) { while (a > M_PI) a -= 2*M_PI; while (a < -M_PI) a += 2*M_PI; return a; };
    const double yawA = A.pose.Rot().Yaw();
    const double yawB = B.pose.Rot().Yaw();
    const double dy   = wrap(yawB - yawA);
    double yaw        = wrap(yawA + u * dy);

    // Optional forward-axis correction in degrees
    yaw += this->yawOffsetDeg * M_PI / 180.0;

    const gz::math::Pose3d P(pos.X(), pos.Y(), pos.Z(), 0, 0, yaw);

    // Command world pose (create or update WorldPoseCmd component)
    if (auto *cmd = _ecm.Component<components::WorldPoseCmd>(this->modelEntity))
      cmd->Data() = P;
    else
      _ecm.CreateComponent(this->modelEntity, components::WorldPoseCmd(P));
  }

private:
  Entity modelEntity{kNullEntity};
  std::vector<Keyframe> keys;

  bool   loop{true};
  bool   autoStart{true};
  bool   started{false};
  double duration{0.0};
  double yawOffsetDeg{0.0};

  // sim8 uses durations for sim time
  std::chrono::steady_clock::duration startSim{};
};

GZ_ADD_PLUGIN(ModelWaypointMover,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ModelWaypointMover, "model_waypoint_mover")
