#include <ros/ros.h>
#include "rmi_driver/driver.h"

using namespace rmi_driver;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "keba_driver");

  ros::NodeHandle nh;

  rmi_driver::Driver driver;

  driver.start();

  ros::spin();

  return 0;
}
