#ifndef ARP_HARDWARE_H
#define ARP_HARDWARE_H

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "arp_base/roboteq_driver/controller.h"
#include "arp_base/roboteq_driver/channel.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace arp_base
{

class ArpHardware : public hardware_interface::RobotHW
{
public:

  const static uint8_t NUM_CONTROLLERS = 2;

  ArpHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);

  void updateJointsFromHardware();

  void writeCommandsToHardware();

  void initReadFromHardware(int index);

private:
  void registerControlInterfaces();

  void resetTravelOffset();

  void setupChannel(int index,const char* position);

  void connect(int index,const char* port,const char* position);

  double linearToAngular(const double &travel) const;

  double angularToLinear(const double &angle) const;

  void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

  ros::NodeHandle nh_,private_nh_;

  // ROS Control Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // ROS Parameters
  double wheel_diameter_, max_accel_, max_speed_;

  double polling_timeout_;

  //Robotec controlers

  roboteq::Controller controller[2];

  // Joint structure
  struct Joint
  {
    double position;
    double position_offset;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
  } joints_[4];
};
}

#endif // ARP_HARDWARE_H
