#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "keba_rmi_plugin/commands_keba.h"

PLUGINLIB_EXPORT_CLASS(keba_rmi_plugin::KebaCommandRegister, rmi_driver::CommandRegister);
