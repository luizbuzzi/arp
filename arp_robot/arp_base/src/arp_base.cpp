#include "arp_base/arp_hardware.h"
#include "ros/callback_queue.h"
#include "controller_manager/controller_manager.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

void controlLoop(arp_base::ArpHardware &arp,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time=this_time;

  arp.updateJointsFromHardware();
  cm.update(ros::Time::now(),elapsed);
  arp.writeCommandsToHardware();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "arp_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency;

  private_nh.param<double>("control_frequency",control_frequency,10.0);

  arp_base::ArpHardware arp(nh,private_nh);
  controller_manager::ControllerManager cm(&arp,nh);

  ros::CallbackQueue arp_queue;
  ros::AsyncSpinner arp_spinner(1, &arp_queue);

  time_source::time_point last_time = time_source::now();

  ros::TimerOptions control_timer(ros::Duration(1/control_frequency),
                                  boost::bind(controlLoop,boost::ref(arp),boost::ref(cm),
                                             boost::ref(last_time)),&arp_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  arp_spinner.start();

  ros::spin();

  return 0;
}
