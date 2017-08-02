
#include <ros/ros.h>
#include "driver.h"

using namespace keba_rmi_driver;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "keba_driver");

    ros::NodeHandle nh;

    keba_rmi_driver::Driver driver;


    driver.addConnection("192.168.100.100", 30000);
    ros::spin();

    return 0;
}
