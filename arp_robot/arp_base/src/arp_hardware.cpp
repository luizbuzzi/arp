#include "arp_base/arp_hardware.h"
#include <boost/assign/list_of.hpp>
#include "arp_base/roboteq_driver/controller.h"
#include "arp_base/roboteq_driver/channel.h"

namespace
{
const double RAD_STEP=1.308996939;

}

namespace arp_base
{

/**
 * @brief ArpHardware::ArpHardware
 * @param nh
 * @param private_nh
 * Inicilaliza o hardware do ARP
 */
ArpHardware::ArpHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{
  private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.3);
  private_nh_.param<double>("max_accel", max_accel_, 5.0);
  private_nh_.param<double>("max_speed", max_speed_, 10.0);
  private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

  std::string port[NUM_CONTROLLERS];
  int32_t baund[NUM_CONTROLLERS];

  for (int i = 0; i < NUM_CONTROLLERS; i++)
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
    controller[i].startScript();
  }
  resetTravelOffset();
  registerControlInterfaces();
}

ArpHardware::~ArpHardware()
{
  for (int i = 0; i < NUM_CONTROLLERS; i++)
  {
    controller[i].setEstop();
    controller[i].stopScript();
  }
}

void ArpHardware::updateJointsFromHardware()
{
  for (int i = 0; i < NUM_CONTROLLERS * 2; i++)
  {
    double delta =
        controller[i/2].getChanels()[i % 2]->getFeedBack().measured_position -
        joints_[i].position - joints_[i].position_offset;

    // detecta perda de dados do encoder
    if (std::abs(delta) < 1.0)
    {
      joints_[i].position += delta;
    }
    else
    {
      joints_[i].position_offset += delta;
      ROS_DEBUG("Dropping overflow measurement from encoder");
    }
    joints_[i].velocity = controller[i/2].getChanels()[i % 2]->getFeedBack().measured_velocity;
  }
  //ROS_INFO("0: %f; 1: %f; 2: %f; 3: %f;", joints_[0].position,
  //joints_[1].position, joints_[2].position, joints_[3].position);
}

void ArpHardware::writeCommandsToHardware()
{
  for (int i = 0; i < NUM_CONTROLLERS * 2; i++)
  {
    controller[i/2].getChanels()[i%2]->cmdCallback(0, velocityDiscretizationFromController(joints_[i].velocity_command));
  }
}

void ArpHardware::registerControlInterfaces()
{
  ros::V_string joint_names = boost::assign::list_of("front_right_wheel")(
      "back_right_wheel")("front_left_wheel")("back_left_wheel");
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
  for (int i = 0; i < NUM_CONTROLLERS * 2; i++)
  {
    joints_[i].position_offset =
        controller[i/2].getChanels()[i % 2]->getFeedBack().measured_position;
  }
}

double ArpHardware::velocityDiscretizationFromController(double velocity)
{
  int velocity_ = velocity/RAD_STEP;
  return velocity_ * RAD_STEP;
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

  while (ros::ok())
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

void ArpHardware::initReadFromHardware(int index)
{
  if (controller[index].connected())
  {
    controller[index].spinOnce();
  }
  else
  {
    ROS_DEBUG("Problem connecting to serial device.");
    ROS_ERROR_STREAM_ONCE("Problem connecting to serial device.");
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
