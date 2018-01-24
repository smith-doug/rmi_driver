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
#include "rmi_driver/util.h"

namespace rmi_driver
{
Driver::Driver() : work_(io_service_), logger_("DRIVER", "/")
{
  ros::NodeHandle nh;
  config_.loadConfig(nh);
}

void Driver::loadPlugin(const ConnectionConfig &con_cfg, CmdRegLoaderPtr &cmd_reg_loader,
                        CommandRegisterPtr &cmd_register)
{
  cmd_register.reset();

  cmd_reg_loader.reset(new CmdRegLoader(con_cfg.rmi_plugin_package_, "rmi_driver::"
                                                                     "CommandRegister"));

  // cmh_loader->createInstance() returns a boost::shared_ptr but I want a std one.
  cmd_register = cmd_reg_loader->createUniqueInstance(con_cfg.rmi_plugin_lookup_name_);
  cmd_register->initialize(con_cfg.joints_);
  logger_.INFO() << "Loaded the plugin successfully";

  // Display some info about the loaded plugin
  logger_.INFO() << "There are " << cmd_register->handlers().size() << " handlers registered";
  for (auto &cmh : cmd_register->handlers())
  {
    logger_.INFO() << *cmh;
  }
}

void Driver::start()
{
  io_service_thread_ = std::thread([&]() { io_service_.run(); });
  util::setThreadName(io_service_thread_, "io_svc_thr");
  util::setThreadName("driver_thr");

  logger_.INFO() << "There are " << config_.connections_.size() << " connections";
  for (auto &&con_cfg : config_.connections_)
  {
    logger_.INFO() << "Loading plugin: " << con_cfg.rmi_plugin_package_;
    try
    {
      // Will be stored in the Connector.  Making it here to keep Plugin loading stuff out of Connector.
      CmdRegLoaderPtr cmd_reg_loader;
      CommandRegisterPtr cmd_register;

      loadPlugin(con_cfg, cmd_reg_loader, cmd_register);

      // Add the connection from the current config
      this->addConnection(con_cfg.ns_, con_cfg.ip_address_, con_cfg.port_, con_cfg.joints_, cmd_reg_loader,
                          cmd_register);
    }
    catch (pluginlib::PluginlibException &ex)
    {
      logger_.ERROR() << "The plugin failed to load for some reason. Error: %s", ex.what();
    }
  }

  // Create ros publishers and subscribers
  joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  // command_list_sub_ = nh_.subscribe("command_list", 1, &Driver::subCB_CommandList, this);

  // Publish joint states.  Will aggregate multiple robots.
  pub_thread_ = std::thread(&Driver::publishJointState, this);
  util::setThreadName(pub_thread_, "pub_jt_state");

  return;
}

void Driver::addConnection(std::string ns, std::string host, int port, std::vector<std::string> joint_names,
                           CmdRegLoaderPtr cmd_reg_loader, CommandRegisterPtr cmd_register)
{
  conn_num_++;

  // Make a new Connector and add it
  auto shared = std::make_shared<Connector>(ns, io_service_, host, port, joint_names, cmd_reg_loader, cmd_register,
                                            config_.clear_commands_on_error_);
  conn_map_.emplace(conn_num_, shared);

  if (config_.use_rmi_driver_jta_)
  {
    auto jta = std::make_shared<JointTrajectoryAction>(ns, joint_names, cmd_register->getJtaCommandHandler());
    jta_map_.emplace(conn_num_, jta);
  }
  else
  {
    logger_.WARN() << "use_rmi_driver_jta disabled. " << ns << "/joint_trajectory_action will not be launched.";
  }

  auto &conn = conn_map_.at(conn_num_);
  conn->connect();
}

void Driver::publishJointState()
{
  ros::Rate pub_rate(config_.publishing_rate_);

  logger_.INFO() << "Driver pub starting";

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
      stateFull.velocity.insert(stateFull.velocity.begin(), 0, lastState.name.size());
      stateFull.effort.insert(stateFull.effort.begin(), 0, lastState.name.size());

      // Publish the individual state topics for this connection (tool_frame)
      conn.second->publishState();
    }
    stateFull.header.stamp = ros::Time::now();
    joint_state_publisher_.publish(stateFull);
    pub_rate.sleep();
  }
}

}  // namespace rmi_driver
