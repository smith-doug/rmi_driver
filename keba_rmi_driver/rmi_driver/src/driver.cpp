/*
 * Copyright (c) 2017, Doug Smith
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *  Created on: Aug 1, 2017
 *      Author: Doug Smith
 */

#include "rmi_driver/driver.h"
#include <iostream>

namespace keba_rmi_driver
{
Driver::Driver()
{
  ros::NodeHandle nh;
  config_.loadConfig(nh);
}

void Driver::start()
{
  //Hardcoded to 1 connection for now

  //Get the config for the connection.  Should be a loop in the future.
  auto &con_cfg = config_.connections_[0];

  //Load the specified plugin.  This should be done in the individual connection in the future.
  ROS_INFO_STREAM("Loading plugin: " << con_cfg.rmi_plugin_package_);
  cmh_loader.reset(
      new pluginlib::ClassLoader<CommandRegister>(con_cfg.rmi_plugin_package_, "keba_rmi_driver::CommandRegister"));
  try
  {
    cmd_register_ = cmh_loader->createUniqueInstance(con_cfg.rmi_plugin_lookup_name_);
    cmd_register_->registerCommands();

    ROS_INFO_STREAM("Loaded the plugin successfully");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  //Display some info about the loaded plugin
  ROS_INFO_STREAM("There are " << cmd_register_->handlers().size() << " handlers registered");
  for (auto &cmh : cmd_register_->handlers())
  {
    ROS_INFO_STREAM(*cmh);
  }

  //Add the connection from the current config
  this->addConnection(con_cfg.ip_address_, 30000, cmd_register_);

  //Create ros publishers and subscribers
  joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  command_list_sub_ = nh.subscribe("command_list", 1, &Driver::subCB_CommandList, this);

  //Publish joint states.  @todo aggregate multiple robots
  pub_thread_ = std::thread(&Driver::publishJointState, this);

  return;

}

bool Driver::commandListCb(const robot_movement_interface::CommandList &msg)
{

  std::cout << "Got a command" << std::endl;

  auto &conn = conn_map_.begin()->second;

  if (conn_map_.begin() == conn_map_.end() || conn_map_.begin()->second == NULL)
    return false;

  if (msg.replace_previous_commands)
    conn->clearCommands();

  for (auto &msg_cmd : msg.commands)
  {
    std::string command_str = "";
    std::string command_params = "";

    std::ostringstream oss;

    auto handler = cmd_register_->findHandler(msg_cmd);

    if (handler)
    {
      std::cout << "Found cmd handler\n";
      Command telnet_command;
      handler->processMsg(msg_cmd, telnet_command);
      conn->addCommand(telnet_command);
      continue;
    }
    else
    {
      std::cout << "Failed to find cmd handler\n";
    }
  }

  return true;

}

void Driver::addConnection(std::string host, int port, std::shared_ptr<CommandRegister> commands)
{
  conn_num_++;

  std::vector<std::string> joint_names {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                        "wrist_2_joint", "wrist_3_joint", "rail_to_base"};

  auto shared = std::make_shared<Connector>(io_service_, host, port, joint_names);
  conn_map_.emplace(conn_num_, shared);

  auto &conn = conn_map_.at(conn_num_);
  conn->connect();
  //conn.connect(host, port);
}

void Driver::publishJointState()
{
  ros::Rate pub_rate(30);
  std::cout << "Driver pub" << std::endl;
  ROS_INFO_NAMED("Driver", "publishJointState");
  while (ros::ok())
  {
    for (auto &conn : conn_map_)
    {
      auto lastState = conn.second->getLastJointState();
      joint_state_publisher_.publish(lastState);
    }
    pub_rate.sleep();
  }
}

void Driver::loadConfig()
{
  ros::NodeHandle nh("~");

}

} // namespace keba_rmi_driver

