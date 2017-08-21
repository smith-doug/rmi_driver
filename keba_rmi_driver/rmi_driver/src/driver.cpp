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
#include <future>
#include <iostream>

namespace rmi_driver
{
Driver::Driver() : work_(io_service_)
{
  // boost::asio::io_service work(io_service_);
  io_service_thread_ = std::thread([&]() { io_service_.run(); });

  ros::NodeHandle nh;
  config_.loadConfig(nh);
}

void Driver::start()
{
  // Hardcoded to 1 connection for now

  // Get the config for the connection.  Should be a loop in the future.
  auto &con_cfg = config_.connections_[0];

  // Load the specified plugin.  This should be done in the individual connection in the future.
  ROS_INFO_STREAM("Loading plugin: " << con_cfg.rmi_plugin_package_);
  cmh_loader_.reset(new pluginlib::ClassLoader<CommandRegister>(con_cfg.rmi_plugin_package_, "rmi_driver::"
                                                                                             "CommandRegister"));
  try
  {
    // cmh_loader->createInstance() returns a boost::shared_ptr but I want a std one.
    CommandRegisterPtr cmd_register = cmh_loader_->createUniqueInstance(con_cfg.rmi_plugin_lookup_name_);
    cmd_register->registerCommands();

    ROS_INFO_STREAM("Loaded the plugin successfully");

    // Display some info about the loaded plugin
    ROS_INFO_STREAM("There are " << cmd_register->handlers().size() << " handlers registered");
    for (auto &cmh : cmd_register->handlers())
    {
      ROS_INFO_STREAM(*cmh);
    }

    // Add the connection from the current config

    std::vector<std::string> joint_names{ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                          "wrist_2_joint",      "wrist_3_joint",       "rail_to_base" };
    this->addConnection(con_cfg.ip_address_, con_cfg.port_, cmd_register, joint_names);

    // Add a second hardcoded connection for now
    joint_names = { "rob2_shoulder_pan_joint", "rob2_shoulder_lift_joint", "rob2_elbow_joint",
                    "rob2_wrist_1_joint",      "rob2_wrist_2_joint",       "rob2_wrist_3_joint" };

    this->addConnection(con_cfg.ip_address_, 30002, cmd_register, joint_names);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  // Create ros publishers and subscribers
  joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  command_list_sub_ = nh_.subscribe("command_list", 1, &Driver::subCB_CommandList, this);

  // Publish joint states.  @todo aggregate multiple robots
  pub_thread_ = std::thread(&Driver::publishJointState, this);

  return;
}

bool Driver::commandListCb(const robot_movement_interface::CommandList &msg)
{
  ROS_INFO_STREAM("Driver::commandListCb Got a command with " << msg.commands.size() << " commands");

  // Hardcoded to robot 1.
  auto conn_entry = conn_map_.find(1);  // conn_map_.begin()->second;
  if (conn_entry == conn_map_.end() || !conn_entry->second)
    return false;

  auto &conn = conn_entry->second;

  auto cmd_register = conn->getCommandRegister();

  if (msg.replace_previous_commands)
    conn->clearCommands();

  for (auto &&msg_cmd : msg.commands)
  {
    std::string command_str = "";
    std::string command_params = "";

    std::ostringstream oss;

    auto handler = cmd_register->findHandler(msg_cmd);

    if (handler)
    {
      ROS_INFO_STREAM("Found cmd handler: " << handler->getName());
      // Command telnet_command;

      // Create a new CommandPtr with the found handler
      auto telnet_command_ptr = handler->processMsg(msg_cmd);
      if (!telnet_command_ptr)
      {
        ROS_WARN_STREAM("Driver::commandListCb got a null telnet_command_ptr");
        continue;
      }

      // Standard Cmds get added to the queue
      if (telnet_command_ptr->getType() == Command::CommandType::Cmd)
      {
        conn->addCommand(telnet_command_ptr);
      }
      else
      {
        ROS_INFO_STREAM("Got a high priority command via a message: " << telnet_command_ptr->getCommand());

        // Call cancelSocketCmd with async.  It will block while it tries to acquire the mutex.
        auto fut = std::async(std::launch::async, &Connector::cancelSocketCmd, conn, 50);

        std::string send_response = conn->sendCommand(*telnet_command_ptr);
        boost::trim_right(send_response);

        ROS_INFO_STREAM("High priority response: " << send_response);
        fut.wait();
      }
      continue;
    }
    else
    {
      std::cout << "Failed to find cmd handler\n";
    }
  }

  return true;
}

void Driver::addConnection(std::string host, int port, CommandRegisterPtr commands,
                           std::vector<std::string> joint_names)
{
  conn_num_++;

  //  std::vector<std::string> joint_names {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
  //                                        "wrist_2_joint", "wrist_3_joint", "rail_to_base"};

  auto shared = std::make_shared<Connector>(io_service_, host, port, joint_names, commands);
  conn_map_.emplace(conn_num_, shared);

  auto &conn = conn_map_.at(conn_num_);
  conn->connect();
  // conn.connect(host, port);
}

void Driver::publishJointState()
{
  ros::Rate pub_rate(30);
  std::cout << "Driver pub" << std::endl;
  ROS_INFO_NAMED("Driver", "publishJointState");

  sensor_msgs::JointState stateFull;
  while (ros::ok())
  {
    stateFull = sensor_msgs::JointState();
    for (auto &&conn : conn_map_)
    {
      auto lastState = conn.second->getLastJointState();
      stateFull.header = lastState.header;
      stateFull.position.insert(stateFull.position.end(), lastState.position.begin(), lastState.position.end());
      stateFull.name.insert(stateFull.name.end(), lastState.name.begin(), lastState.name.end());
    }

    joint_state_publisher_.publish(stateFull);
    pub_rate.sleep();
  }
}

void Driver::loadConfig()
{
  ros::NodeHandle nh("~");
}

}  // namespace rmi_driver
