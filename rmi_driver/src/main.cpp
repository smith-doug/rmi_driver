#include <ros/ros.h>
#include "rmi_driver/driver.h"
#include "rmi_driver/joint_trajectory_action.h"

using namespace rmi_driver;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rmi_driver");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ROS_INFO_STREAM("rmi_driver starting after 1 second delay");
  ros::Duration(1).sleep();  // Sleep to allow rqt_console to detect the new node

  rmi_driver::Driver driver;

  driver.start();

  ros::waitForShutdown();
  // ros::spin();

  return 0;
}
