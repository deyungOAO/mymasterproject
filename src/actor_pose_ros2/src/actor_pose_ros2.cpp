#include <unordered_map>
#include <string>
#include <memory>
#include <cmath>

#// Gazebo / gz-sim
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>   // Pose component (WorldPose isn’t a separate header)
#include <gz/sim/Util.hh>              // worldPose(entity, ecm)
#include <gz/sim/Types.hh>        // UpdateInfo lives here
#include <chrono> 

// gz-math
#include <gz/math/Quaternion.hh>
#include <gz/math/Pose3.hh>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
//#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/transform_broadcaster.h>


using namespace gz;
using namespace gz::sim;

class ActorPoseRos2 final
  : public System,
    public ISystemConfigure,
    public ISystemPostUpdate
{
public:
  void Configure(const Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &,
                 EventManager &) override
  {
    parentFrame_       = _sdf->Get<std::string>("base_frame", "world").first;
    namePrefix_        = _sdf->Get<std::string>("name_prefix", "actor_").first;
    poseTopic_         = _sdf->Get<std::string>("pose_topic", "/actors/poses").first;
    posePrefix_        = _sdf->Get<std::string>("pose_topic_prefix", "/actors/").first;
    publishTF_         = _sdf->Get<bool>("publish_tf", true).first;
    actorYawOffsetDeg_ = _sdf->Get<double>("actor_yaw_offset_deg", -90.0).first;

    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("actor_pose_ros2_NO_CLOCK");
    posesPub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(poseTopic_, 10);
    //clockPub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    if (publishTF_) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    RCLCPP_INFO(node_->get_logger(),
      "actor_pose_ros2 configured. base_frame='%s', name_prefix='%s', pose_topic='%s'",
      parentFrame_.c_str(), namePrefix_.c_str(), poseTopic_.c_str());
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override
{
  // Convert Gazebo sim time -> ROS time fields
  using namespace std::chrono;
  const auto ns   = duration_cast<nanoseconds>(_info.simTime).count();
  const int32_t  sec  = static_cast<int32_t>(ns / 1000000000LL);
  const uint32_t nsec = static_cast<uint32_t>(ns % 1000000000LL);

  geometry_msgs::msg::PoseArray pa;
  pa.header.frame_id   = parentFrame_;
  pa.header.stamp.sec  = sec;
  pa.header.stamp.nanosec = nsec;

  const double yawRad = actorYawOffsetDeg_ * M_PI / 180.0;
  gz::math::Quaterniond yawOff(0, 0, yawRad);

  // Iterate named entities with a Pose component
  _ecm.Each<gz::sim::components::Name, gz::sim::components::Pose>(
    [&](const gz::sim::Entity &ent,
        const gz::sim::components::Name *name,
        const gz::sim::components::Pose *) -> bool
    {
      const std::string &n = name->Data();
      if (n.rfind(namePrefix_, 0) != 0)   // not starting with prefix
        return true;

      // World pose
      const gz::math::Pose3d W = gz::sim::worldPose(ent, _ecm);

      // PoseStamped (create INSIDE the loop)
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id   = parentFrame_;
      ps.header.stamp.sec  = sec;
      ps.header.stamp.nanosec = nsec;
      ps.pose.position.x = W.Pos().X();
      ps.pose.position.y = W.Pos().Y();
      ps.pose.position.z = W.Pos().Z();

      auto qOut = W.Rot() * yawOff;
      ps.pose.orientation.x = qOut.X();
      ps.pose.orientation.y = qOut.Y();
      ps.pose.orientation.z = qOut.Z();
      ps.pose.orientation.w = qOut.W();

      pa.poses.push_back(ps.pose);

      // Per-actor topic (lazy create)
      auto &pub = actorPub_[n];
      if (!pub) {
        std::string topic = posePrefix_ + n + "/pose";
        for (auto &ch : topic) if (ch == ' ') ch = '_';
        pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
      }
      pub->publish(ps);

      // TF (also create INSIDE the loop)
      if (publishTF_ && tf_) {
        geometry_msgs::msg::TransformStamped tfm;
        tfm.header.frame_id   = parentFrame_;
        tfm.header.stamp.sec  = sec;
        tfm.header.stamp.nanosec = nsec;
        tfm.child_frame_id = n;
        tfm.transform.translation.x = ps.pose.position.x;
        tfm.transform.translation.y = ps.pose.position.y;
        tfm.transform.translation.z = ps.pose.position.z;
        tfm.transform.rotation      = ps.pose.orientation;
        tf_->sendTransform(tfm);
      }
      return true;
    });

  // Aggregate PoseArray
  posesPub_->publish(pa);

  // Publish /clock from sim time (no bridge needed)
  //rosgraph_msgs::msg::Clock clk;
  //clk.clock.sec    = sec;
  //clk.clock.nanosec= nsec;
  //clockPub_->publish(clk);

  rclcpp::spin_some(node_);
}

private:
  static bool startsWith(const std::string &s, const std::string &p) {
    return s.rfind(p, 0) == 0;
  }

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posesPub_;
  //rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub_;
  std::unordered_map<std::string,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> actorPub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;

  // Params
  std::string parentFrame_{"world"};
  std::string namePrefix_{"actor_"};
  std::string poseTopic_{"/actors/poses"};
  std::string posePrefix_{"/actors/"};
  bool publishTF_{true};
  double actorYawOffsetDeg_{-90.0};
};

GZ_ADD_PLUGIN(ActorPoseRos2,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(ActorPoseRos2, "actor_pose_ros2")
