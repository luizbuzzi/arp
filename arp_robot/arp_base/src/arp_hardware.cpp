#include "arp_base/arp_hardware.h"
#include <boost/assign/list_of.hpp>
#include "arp_base/roboteq_driver/controller.h"

namespace
{
const uint8_t CONTROLLER_A = 0, CONTROLLER_B = 1;
}

namespace arp_base
{

ArpHardware::ArpHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{
  private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.3);
  private_nh_.param<double>("max_accel", max_accel_, 5.0);
  private_nh_.param<double>("max_speed", max_speed_, 10.0);
  private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

  std::string port[2];
  int32_t baund[2];

  for (int i = 0; i < 2; i++)
  {
    private_nh_.param<std::string>(
        "port_" + boost::lexical_cast<std::string>(i), port[i],
        "/dev/ttyACM" + boost::lexical_cast<std::string>(i));
    private_nh_.param<int32_t>("baund_" + boost::lexical_cast<std::string>(i),
                               baund[i], 115200);
    controller[i].controlerInit(port[i].c_str(), baund[i]);
    if (i % 2)
    {
      setupChannel(i, "left");
      connect(i, port[i].c_str(), "left");
    }
    else
    {
      setupChannel(i, "right");
      connect(i, port[i].c_str(), "right");
    }
  }
  resetTravelOffset();
  registerControlInterfaces();
}

void ArpHardware::updateJointsFromHardware()
{
  controller[0].spinOnce();
  controller[1].spinOnce();
  joints_[0].position = controller[0].getChanels()[0]->getFeedBack().ticks;
  joints_[1].position = controller[1].getChanels()[0]->getFeedBack().ticks;
  joints_[2].position = controller[0].getChanels()[1]->getFeedBack().ticks;
  joints_[3].position = controller[1].getChanels()[1]->getFeedBack().ticks;
  ROS_INFO("0: %f; 1: %f; 2: %f; 3: %f;", joints_[0].position,
           joints_[1].position, joints_[2].position, joints_[3].position);
}

void ArpHardware::writeCommandsToHardware()
{
  double diff_speed_left = angularToLinear(joints_[0].velocity_command);
  double diff_speed_right = angularToLinear(joints_[1].velocity_command);

  limitDifferentialSpeed(diff_speed_left, diff_speed_right);

  joints_[0].velocity = diff_speed_left;
  joints_[1].velocity = diff_speed_right;
  joints_[2].velocity = diff_speed_left;
  joints_[3].velocity = diff_speed_right;
}

void ArpHardware::registerControlInterfaces()
{
  ros::V_string joint_names = boost::assign::list_of("front_left_wheel")(
      "front_right_wheel")("back_left_wheel")("back_right_wheel");
  for (int i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(
        joint_names[i], &joints_[i].position, &joints_[i].velocity,
        &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle,
                                                 &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
}

void ArpHardware::resetTravelOffset()
{
  joints_[0].position_offset =
      controller[0].getChanels()[0]->getFeedBack().ticks;
  joints_[1].position_offset =
      controller[1].getChanels()[1]->getFeedBack().ticks;
  joints_[2].position_offset =
      controller[0].getChanels()[0]->getFeedBack().ticks;
  joints_[3].position_offset =
      controller[1].getChanels()[1]->getFeedBack().ticks;
}

void ArpHardware::setupChannel(int index, const char* position)
{
  controller[index].addChannel(new roboteq::Channel(
      1, "front_" + boost::lexical_cast<std::string>(position) + "_wheel",
      &controller[index]));
  controller[index].addChannel(new roboteq::Channel(
      2, "back_" + boost::lexical_cast<std::string>(position) + "_wheel",
      &controller[index]));
}

void ArpHardware::connect(int index, const char* port, const char* position)
{
  ROS_DEBUG("Attempting connection to %s for %s wheels", port, position);

  while (1)
  {
    controller[index].connect();

    if (controller[index].connected())
    {
      ROS_DEBUG("Connection successful to %s for %s wheels", port, position);
      ROS_INFO("Connection successful to %s for %s wheels", port, position);
      break;
    }
    else
    {
      ROS_DEBUG("Problem connecting to serial device.");
      ROS_ERROR_STREAM_ONCE("Problem connecting to "
                            << position
                            << " controller.Try again in 1 second.");
      sleep(1);
    }
  }
}

double ArpHardware::linearToAngular(const double& travel) const
{
  return travel / wheel_diameter_ * 2;
}

double ArpHardware::angularToLinear(const double& angle) const
{
  return angle * wheel_diameter_ / 2;
}

void ArpHardware::limitDifferentialSpeed(double& travel_speed_left,
                                         double& travel_speed_right)
{
  double large_speed =
      std::max(std::abs(travel_speed_left), std::abs(travel_speed_right));

  if (large_speed > max_speed_)
  {
    travel_speed_left *= max_speed_ / large_speed;
    travel_speed_right *= max_speed_ / large_speed;
  }
}
}
