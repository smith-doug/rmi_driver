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

#ifndef INCLUDE_CONNECTOR_H_
#define INCLUDE_CONNECTOR_H_

#include <iiwa_driver/StringCommand.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>
#include <ros/ros.h>
#include "rmi_driver/commands.h"

#include <sensor_msgs/JointState.h>

#include <boost/asio.hpp>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace rmi_driver
{
class Connector
{
  typedef std::vector<std::string> StringVec;

public:
  Connector(boost::asio::io_service& io_service, std::string host, int port, StringVec joint_names,
            CommandRegisterPtr cmd_register);

  bool connect();
  bool connect(std::string host, int port);

  bool connectCmd(std::string host, int port);

  /**
   * Sends a command.  It will choose the socket to used based on the command type.
   * @param command
   * @return the reply from the socket.
   */
  std::string sendCommand(const Command& command);

  /**
   * Adds a command to the queue.  Currently only takes Cmd type
   * @param command
   */
  void addCommand(CommandPtr command);

  void clearCommands();

  sensor_msgs::JointState getLastJointState()
  {
    return last_joint_state_;
  }

  void cancelSocketGet()
  {
    socket_get_.cancel();
  }

  void cancelSocketCmd()
  {
    socket_cmd_.cancel();
  }

  CommandRegisterPtr getCommandRegister()
  {
    return cmd_register_;
  }

protected:
  void commandThread();

  void cmdThread();

  void getThread();

  // Socket used for motion commands that may block.  Default port 30000
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_cmd_ptr_;
  boost::asio::ip::tcp::socket socket_cmd_;

  // Socket used for "instant" commands that can't block.  Default port socket_cmd_ + 1
  boost::asio::ip::tcp::socket socket_get_;

  std::mutex command_list_mutex_;

  std::mutex socket_cmd_mutex_;
  std::mutex socket_get_mutex_;

  std::promise<int> get_promise_;
  std::future<int> get_future_;

  // boost::asio::streambuf get_buff_;
  boost::asio::mutable_buffer get_buff_;

  std::string host_;
  int port_;

  boost::asio::io_service& io_service_;

  boost::asio::io_service io_service_test_;
  std::thread io_service_test_thread_;

  boost::asio::io_service local_io_service_;

  std::queue<CommandPtr> command_list_;

  std::thread get_thread_;
  std::thread cmd_thread_;

  sensor_msgs::JointState last_joint_state_;

  std::vector<std::string> joint_names_;

  CommandRegisterPtr cmd_register_;
};

}  // namespace rmi_driver

#endif /* INCLUDE_CONNECTOR_H_ */
