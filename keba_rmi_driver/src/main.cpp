#include <ros/ros.h>
#include "keba_rmi_driver/driver.h"

using namespace keba_rmi_driver;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "keba_driver");

  ros::NodeHandle nh;

  keba_rmi_driver::Driver driver;

  driver.start();

  ros::spin();

  return 0;
}
