#include "SwerveActor.hh"
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <std_msgs/msg/float32.hpp>

GZ_ADD_PLUGIN(swerve_actor::SwerveActor,
              gz::sim::System,
              swerve_actor::SwerveActor::ISystemConfigure)

using namespace swerve_actor;

swerve_actor::SwerveActor::SwerveActor()
{

}

swerve_actor::SwerveActor::~SwerveActor()
{
  rclcpp::shutdown();
}

void swerve_actor::SwerveActor::Configure(const gz::sim::Entity &,
                                          const std::shared_ptr<const sdf::Element> &,
                                          gz::sim::EntityComponentManager &_ecm,
                                          gz::sim::EventManager &)
{

  gzdbg << "SwerveActor loaded successfully!" << std::endl;

  // Initialize joint variables
  this->driveJoint1 = 0.0;
  this->driveJoint2 = 0.0;
  this->driveJoint3 = 0.0;
  this->driveJoint4 = 0.0;

  this->pivotJoint1 = 0.0;
  this->pivotJoint2 = 0.0;
  this->pivotJoint3 = 0.0;
  this->pivotJoint4 = 0.0;

  // Get Gazebo things set up so I can reference the Gazebo environment
  this->parent = _parent;
  gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "DiffDrive" ) );



  gzdbg << "ROS 2 subscribers created for drive and pivot joints." << std::endl;
}

