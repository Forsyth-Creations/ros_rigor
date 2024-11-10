#ifndef SWERVE_ACTOR_HH
#define SWERVE_ACTOR_HH

#include <gz/sim/System.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace swerve_actor
{
  class SwerveActor : public gz::sim::System, public gz::sim::ISystemConfigure
  {
  public:
    SwerveActor();
    ~SwerveActor() override;

    // Method called once when the plugin is loaded
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override;

  private:
    // Variables for drive and pivot joints
    float driveJoint1;
    float driveJoint2;
    float driveJoint3;
    float driveJoint4;

    float pivotJoint1;
    float pivotJoint2;
    float pivotJoint3;
    float pivotJoint4;
  };
}

#endif // SWERVE_ACTOR_HH
