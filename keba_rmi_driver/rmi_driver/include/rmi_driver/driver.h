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

#ifndef INCLUDE_DRIVER_H_
#define INCLUDE_DRIVER_H_

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include "rmi_driver/connector.h"
#include "rmi_driver/commands.h"

//#include "rmi_driver/commands_keba.h"

#include <sensor_msgs/JointState.h>
#include <robot_movement_interface/CommandList.h>

#include <boost/asio.hpp>
#include <unordered_map>

//#include <industrial_utils/param_utils.h>

namespace rmi_driver
{

class DriverConfig
{
public:

  class ConnectionConfig
  {
  public:
    ConnectionConfig()
    {
    }

    std::string ip_address_;
    std::string rmi_plugin_package_;
    std::string rmi_plugin_lookup_name_;
    std::vector<std::string> joints_;

  };

  DriverConfig()
  {
  }


  //Force template deduction to be a string and not a char[] when passed a "" string
  template<typename T>
  struct identity
  {
    typedef T type;
  };

  template<typename T>
  void loadParam(ros::NodeHandle &nh, const std::string &key, T &val, const typename identity<T>::type &def)
  {
    bool loadOk = nh.param<T>(key, val, def);
    if (loadOk)
      std::cout << "Param loaded. ";
    else
      std::cout << "Failed to load, using default. ";

    std::cout << key << " = " << val << std::endl;

  }

  /**
   * This is terrible but I can't get it to load structured data like controller_joint_map right now.
   * @param nh
   */
  void loadConfig(ros::NodeHandle &nh)
  {
    std::cout << "loading config\n";

    std::vector<std::string> sadsad;

    ConnectionConfig cfg;

    loadParam(nh, "/rmi_driver/connection/ip_address", cfg.ip_address_, "192.168.100.100");

    loadParam(nh, "/rmi_driver/connection/rmi_plugin_package", cfg.rmi_plugin_package_, "keba_rmi_plugin");

    loadParam(nh, "/rmi_driver/connection/rmi_plugin_lookup_name", cfg.rmi_plugin_lookup_name_,
              "keba_rmi_plugin::KebaCommandRegister");

    connections_.push_back(cfg);

  }

  std::vector<ConnectionConfig> connections_;

};

class Driver
{
public:
  Driver();

  void start();

  void addConnection(std::string host, int port, std::shared_ptr<CommandRegister> commands);

  void publishJointState();

  void subCB_CommandList(const robot_movement_interface::CommandListConstPtr &msg)
  {
    commandListCb(*msg);
  }

  /**
   * Called when a CommandList message is received.
   *
   * if replace_previous_commands is set, it will clear the queue.
   * It will search through the registered command handlers for each message in the CommandList.
   * If a handler is found, the handler will create a rmi_driver::Command (@todo come up with a better name).
   *
   * For a normal Cmd type, it will add it to the queue to be sent later.
   * For a high priority Get type, it will clear the queue, then
   * send it immediately.   This is useful for things like ABORT.   *
   *
   * @param msg received from the /command_list topic
   * @return true
   */
  bool commandListCb(const robot_movement_interface::CommandList &msg);

  void loadConfig();

  DriverConfig config_;

protected:

  std::unique_ptr<pluginlib::ClassLoader<CommandRegister>> cmh_loader;

  ros::NodeHandle nh;

  std::unordered_map<int32_t, std::shared_ptr<Connector>> conn_map_;

  //Connector connector_;

  int conn_num_ = 0;

  boost::asio::io_service io_service_;

  ros::Subscriber command_list_sub_;

  ros::Publisher joint_state_publisher_;

  std::thread pub_thread_;

  //std::vector<CommandHandler> cmd_handlers_;  //###testing

  std::shared_ptr<CommandRegister> cmd_register_;

};
} //namespace rmi_driver

#endif /* INCLUDE_DRIVER_H_ */
