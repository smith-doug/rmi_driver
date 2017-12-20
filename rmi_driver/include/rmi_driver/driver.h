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

#ifndef INCLUDE_DRIVER_H_
#define INCLUDE_DRIVER_H_

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "rmi_driver/commands.h"
#include "rmi_driver/connector.h"
#include "rmi_driver/joint_trajectory_action.h"
#include "rmi_driver/rmi_config.h"
#include "rmi_driver/rmi_logger.h"

#include <robot_movement_interface/CommandList.h>
#include <robot_movement_interface/Result.h>
#include <sensor_msgs/JointState.h>

#include <boost/asio.hpp>
#include <unordered_map>

//#include <industrial_utils/param_utils.h>

namespace rmi_driver
{
class Driver
{
public:
  Driver();

  void start();

  void addConnection(std::string ns, std::string host, int port, std::vector<std::string> joint_names,
                     CommandRegisterPtr commands, CmhLoaderPtr cmh_loader);

  // void addConnection(ConnectionConfig con_cfg);

  void publishJointState();

  void loadConfig();

  DriverConfig config_;

protected:
  // std::unique_ptr<pluginlib::ClassLoader<CommandRegister>> cmh_loader_;

  ros::NodeHandle nh_;

  std::unordered_map<int32_t, std::shared_ptr<Connector>> conn_map_;

  std::unordered_map<int32_t, std::shared_ptr<JointTrajectoryAction>> jta_map_;

  // Connector connector_;

  int conn_num_ = 0;

  boost::asio::io_service io_service_;
  boost::asio::io_service::work work_;

  // ros::Subscriber command_list_sub_;

  ros::Publisher joint_state_publisher_;

  std::thread pub_thread_;

  std::thread io_service_thread_;

  rmi_log::RmiLogger logger_;

  // std::vector<CommandHandler> cmd_handlers_;  //###testing

  // std::shared_ptr<CommandRegister> cmd_register_;
};

}  // namespace rmi_driver

#endif /* INCLUDE_DRIVER_H_ */
