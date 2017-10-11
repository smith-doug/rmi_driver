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

#ifndef INCLUDE_CONNECTOR_H_
#define INCLUDE_CONNECTOR_H_

#include <iiwa_driver/StringCommand.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>
#include <ros/ros.h>
#include "rmi_driver/commands.h"

#include <robot_movement_interface/EulerFrame.h>
#include <robot_movement_interface/Result.h>

#include <sensor_msgs/JointState.h>

#include <pluginlib/class_loader.h>
#include <boost/asio.hpp>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <robot_movement_interface/CommandList.h>
#include "rmi_driver/rmi_config.h"

namespace rmi_driver
{
using CmhLoader = pluginlib::ClassLoader<CommandRegister>;
using CmhLoaderPtr = std::shared_ptr<pluginlib::ClassLoader<CommandRegister>>;
class Connector
{
  typedef std::vector<std::string> StringVec;

public:
  Connector(std::string ns, boost::asio::io_service& io_service, std::string host, int port, StringVec joint_names,
            CommandRegisterPtr cmd_register, CmhLoaderPtr cmh_loader);

  bool connect();
  bool connect(std::string host, int port);

  // bool connectCmd(std::string host, int port);

  bool connectSocket(std::string host, int port, RobotCommand::CommandType cmd_type);

  /**
   * Sends a command.  It will choose the socket to used based on the command type.
   * @param command
   * @return the reply from the socket.
   */
  std::string sendCommand(const RobotCommand& command);

  /**
   * Adds a command to the queue.  Currently only takes Cmd type
   * @param command
   */
  void addCommand(RobotCommandPtr command);

  void clearCommands();

  void subCB_CommandList(const robot_movement_interface::CommandListConstPtr& msg)
  {
    commandListCb(*msg);
  }

  /**
   * \brief Called when a CommandList message is received.
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
  bool commandListCb(const robot_movement_interface::CommandList& msg);

  sensor_msgs::JointState getLastJointState()
  {
    return last_joint_state_;
  }

  //  void cancelSocketGet()
  //  {
  //    socket_get_.cancel();
  //  }

  /**
   *This function will cancel active socket async commands and launch cmdSocketFlusher if needed.  This is
   * automatically launched  by the driver when a high priority command like ABORT is received as a
   * robot_movement_interface::Command.
   * It will try to lock the socket_cmd_mutex_ to timeout ms.
   * @param timeout ms to try to lock the socket mutex for before actually canceling
   */
  void cancelSocketCmd(int timeout = 50);

  CommandRegisterPtr getCommandRegister()
  {
    return cmd_register_;
  }

  /**
   * This will run and consume any erroneous messages sent by the controller after a cancel.
   * This can happen if
   * 1. A motion command like PTP is sent while the PLC is set to stop at a breakpoint.
   * 2. An ABORT command is issued.  The socket timer will abort, indicating that it didn't get a response.
   * 3. The PLC is unpaused and it sends a "done" or whatever.
   * Without this function running, the next async_read would return immediately with the contents of an old message,
   * and it would be "off by one" forever.
   */
  void cmdSocketFlusher();

protected:
  void commandThread();

  void cmdThread();

  void getThread();

  /**
   * \brief Used by getThread() to create the required RobotCommands for the cyclic updated.
   *
   * @todo Think about this.  Maybe a special command handler type that returns the appropriate message (JointState,
   * etc) should be required.
   * @param command_type Should be GET
   * @param pose_type Currently JOINT_POSITION, TOOL_FRAME, VERSION
   * @return a RobotCommandPtr for specified command and pose type.
   */
  RobotCommandPtr findGetCommandHandler(const std::string& command_type, const std::string& pose_type);

  std::string ns_;  ///< Namespace of this connection

  /// Socket used for motion commands that may block.  Default port 30000
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_cmd_ptr_;
  boost::asio::ip::tcp::socket socket_cmd_;

  /// Socket used for "instant" commands that can't block.  Default port socket_cmd_ + 1
  boost::asio::ip::tcp::socket socket_get_;

  /// Mutex for adding/removing Commands from the queue
  std::mutex command_list_mutex_;

  /// Cmd socket mutex
  std::timed_mutex socket_cmd_mutex_;
  /// Get socket mutex
  std::timed_mutex socket_get_mutex_;

  /// IP address of the robot
  std::string host_;
  /// Port of the robot
  int port_;

  /// asio io service.  Owned by Driver.
  boost::asio::io_service& io_service_;

  /// Queue of all telnet commands to be sent by Connector::cmdThread().
  std::queue<RobotCommandPtr> command_list_;

  /// Receives the robot_movement_interface/CommandList for this namespace
  ros::Subscriber command_list_sub_;
  /// Publishes the robot_movement_interface/Result for this namespace
  ros::Publisher command_result_pub_;
  /// NodeHandle for this namespace
  ros::NodeHandle nh_;

  // std::queue<std::shared_ptr<robot_movement_interface::Result>> command_result_list_;

  std::thread get_thread_;
  std::thread cmd_thread_;

  /// The last known joint state.  Set by getThead() and aggregated by the Driver.
  sensor_msgs::JointState last_joint_state_;

  std::vector<std::string> joint_names_;

  /// The CommandRegister that was loaded by the plugin
  CommandRegisterPtr cmd_register_;

  /// "The ClassLoader must not go out scope while you are using the plugin."  Keep it alive.
  CmhLoaderPtr cmh_loader_;

  ///\brief Used to read and consume and messages sent after a cancel.
  ///
  /// This was happening to me if I paused the PLC, aborted a command, then unpaused the PLC.
  bool flush_socket_cmd_ = false;

  boost::asio::streambuf socket_cmd_flush_buff_;

  bool abort_on_fail_to_find_ = true;  /// @todo Will all robot types have an ABORT command?
};

}  // namespace rmi_driver

#endif /* INCLUDE_CONNECTOR_H_ */
