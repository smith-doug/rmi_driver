/*
 * Copyright (c) 2017, Doug Smith, KEBA Corp
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
  ROS_INFO_STREAM("There are " << config_.connections_.size() << " connections");
  for (auto &&con_cfg : config_.connections_)
  {
    ROS_INFO_STREAM("Loading plugin: " << con_cfg.rmi_plugin_package_);
    try
    {
      // Will be stored in the Connector.  Making it here to keep Plugin loading stuff out of Connector.
      CmhLoaderPtr cmh_loader(new CmhLoader(con_cfg.rmi_plugin_package_, "rmi_driver::"
                                                                         "CommandRegister"));

      // cmh_loader->createInstance() returns a boost::shared_ptr but I want a std one.
      CommandRegisterPtr cmd_register = cmh_loader->createUniqueInstance(con_cfg.rmi_plugin_lookup_name_);
      cmd_register->initialize(con_cfg.joints_);
      ROS_INFO_STREAM("Loaded the plugin successfully");

      // Display some info about the loaded plugin
      ROS_INFO_STREAM("There are " << cmd_register->handlers().size() << " handlers registered");
      for (auto &cmh : cmd_register->handlers())
      {
        ROS_INFO_STREAM(*cmh);
      }

      // Add the connection from the current config
      this->addConnection(con_cfg.ns_, con_cfg.ip_address_, con_cfg.port_, con_cfg.joints_, cmd_register, cmh_loader);
    }
    catch (pluginlib::PluginlibException &ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }
  }

  // Create ros publishers and subscribers
  joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  // command_list_sub_ = nh_.subscribe("command_list", 1, &Driver::subCB_CommandList, this);

  // Publish joint states.  Will aggregate multiple robots.
  pub_thread_ = std::thread(&Driver::publishJointState, this);

  return;
}

void Driver::addConnection(std::string ns, std::string host, int port, std::vector<std::string> joint_names,
                           CommandRegisterPtr commands, CmhLoaderPtr cmh_loader)
{
  conn_num_++;

  // Make a new Connector and add it
  auto shared = std::make_shared<Connector>(ns, io_service_, host, port, joint_names, commands, cmh_loader,
                                            config_.clear_commands_on_error_);
  conn_map_.emplace(conn_num_, shared);

  auto jta = std::make_shared<JointTrajectoryAction>(ns);
  jta_map_.emplace(conn_num_, jta);

  auto &conn = conn_map_.at(conn_num_);
  conn->connect();
}

void Driver::publishJointState()
{
  ros::Rate pub_rate(config_.publishing_rate_);

  ROS_INFO_STREAM(__func__ << "Driver pub starting");
  // ROS_INFO_NAMED("Driver", "publishJointState");

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

      // Publish the individual state topics for this connection (tool_frame)
      conn.second->publishState();
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
