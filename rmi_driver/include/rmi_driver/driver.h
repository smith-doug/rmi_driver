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

namespace rmi_driver
{
class Driver
{
public:
  /**
   * \brief Creates a Driver instance and loads the config
   */
  Driver();

  virtual ~Driver()
  {
    // stop();
  }

  /**
   * \brief Create io_service threads, load plugins, launch Connectors
   */
  void start();

  void stop();

  void run();

  /**
   * \brief Add a Connection to the connection/jta maps and connect
   *
   * @param ns Namespace for this connection
   * @param host ip address of robot
   * @param port base port for Cmd.  Get will be port+1
   * @param joint_names Vector of joint names
   * @param cmd_reg_loader The plugin loader that needs to be stored
   * @param cmd_register CommandRegister loaded from plugin
   */
  void addConnection(std::string ns, std::string host, int port, std::vector<std::string> joint_names,
                     CmdRegLoaderPtr cmd_reg_loader, CommandRegisterPtr cmd_register);

  /**
   * \brief Load a plugin using pluginlib::ClassLoader for 1 connection
   *
   * @param con_cfg [in] The ConnectionConfig
   * @param cmd_reg_loader [out] ClassLoader for pluginlib.  Must be stored to make pluginlib happy.
   * @param cmd_register [out] The actual CommandRegister loaded from the plugin
   */
  void loadPlugin(const ConnectionConfig &con_cfg, CmdRegLoaderPtr &cmd_reg_loader, CommandRegisterPtr &cmd_register);

  /**
   * \brief Aggregate and publish the joint states
   */
  void publishJointState();

  DriverConfig config_;  /// Contains driver params and connection list params

protected:
  ros::NodeHandle nh_;

  /// Map of [conn_num, Connector].  The connection number currently isn't used.
  std::unordered_map<int32_t, std::shared_ptr<Connector>> conn_map_;

  /// Map of [conn_num, jta_handler].  The connection number currently isn't used.  This should really be another node,
  /// but I don't know how to dynamically launch nodes and I don't want to edit launch files when I change the rmi
  /// config.
  std::unordered_map<int32_t, std::shared_ptr<JointTrajectoryAction>> jta_map_;

  int conn_num_ = 0;  /// used for the maps

  boost::asio::io_service io_service_;  /// io_service that will be used for each Connector's asio stuff

  // boost::asio::io_service::work work_;  /// Keep the io_service from dying
  std::unique_ptr<boost::asio::io_service::work> work_;
  std::thread io_service_thread_;  /// calls io_service_.run()

  // std::shared_ptr<std::thread> io_service_thread_;

  ros::Publisher joint_state_publisher_;  /// Publishes aggregated joint states

  std::thread pub_thread_;  /// Aggregates and publishes

  rmi_log::RmiLogger logger_;  /// Easier logging
};

}  // namespace rmi_driver

#endif /* INCLUDE_DRIVER_H_ */
