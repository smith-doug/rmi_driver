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

#include <ros/ros.h>
#include "rmi_driver/commands.h"
#include "rmi_driver/rmi_logger.h"

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
using CmdRegLoader = pluginlib::ClassLoader<CommandRegister>;
using CmdRegLoaderPtr = std::shared_ptr<pluginlib::ClassLoader<CommandRegister>>;

/**
 * \brief The connection to a robot.
 *
 * This class handles the actual socket connections to the robot and processing of messages.  It will listen for
 * robot_movement_interface::CommandList messages, use the plugin provided by Driver to process them into RobotMessages,
 * send them to the robot, and await responses.
 */
class Connector
{
  typedef std::vector<std::string> StringVec;

public:
  Connector(std::string ns, boost::asio::io_service& io_service, std::string host, int port, StringVec joint_names,
            CommandRegisterPtr cmd_register, CmdRegLoaderPtr cmh_loader, bool clear_commands_on_error);

  /**
   * \brief Starts the asynchronous connect methods
   * @return always true
   */
  bool connect();

  /**
   * \brief Starts the asynchronous connect methods
   *
   * This method was originally synchronous and would return the success of the connection.
   * @param host The ip address
   * @param port The port
   * @return Always true
   */
  bool connect(std::string host, int port);

  /**
   * \brief Sends a command.
   *
   * It will choose the socket to used based on the command type.
   * @param command a rmi_driver::RobotCommand to send
   * @return the reply from the socket.
   */
  std::string sendCommand(const RobotCommand& command);

  /**
   * Adds a command to the queue.  Currently only takes Cmd type
   *
   * @param command a RobotCommand shared pointer
   */
  void addCommand(RobotCommandPtr command);

  /**
   * \brief Erase the command queue.
   *
   * This does NOT abort commands that are already executing on the robot.
   */
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
   * If a handler is found, the handler will create a rmi_driver::RobotCommand and store it.  If the entire CommandList
   * is processed successfully, the commands will be added to command_list_ so that cmdThread can send them to the
   * robot.
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

  /**
   * \brief Calls socket::cancel() on the cmd socket if unable to acquire socket_cmd_mutex_ before the timeout expires.
   *
   * This function will cancel active socket async commands and launch cmdSocketFlusher if needed.  This is
   * automatically launched  by the driver when a high priority command like ABORT is received as a
   * robot_movement_interface::Command.  It will try to lock the socket_cmd_mutex_ to timeout ms.  If this succeeds, it
   * will not call cancel or launch the cmdSocketFlusher.
   *
   * @param timeout ms to try to lock the socket mutex for before actually canceling
   */
  void cancelSocketCmd(int timeout = 50);

  /**
   * \brief Get the CommandRegister that was loaded by the plugin
   *
   * @return The CommandRegister shared pointer
   */
  CommandRegisterPtr getCommandRegister()
  {
    return cmd_register_;
  }

  /**
   * \brief Consume any messages received at strange times on the Cmd socket.
   *
   * This will run and consume any erroneous messages sent by the controller after a cancel.
   * This can happen if
   * 1. A motion command like PTP is sent while the PLC is set to stop at a breakpoint.
   * 2. An ABORT command is issued.  The socket timer will abort, indicating that it didn't get a response.  The robot
   * may return a "done" for the previous command.
   * 3. The PLC is unpaused and it sends a "done" or whatever.
   * Without this function running, the next async_read would return immediately with the contents of an old message,
   * and it would be "off by one" forever.
   */
  void cmdSocketFlusher();

  /**
   * \brief Publish any non-aggregated state messages like tool_frame.
   *
   * I already have a publishing thread in Driver.  No real need to make another.  The Driver will call this directly.
   */
  void publishState();

protected:
  /**
   * \brief Monitor command_list_, send command to the robot and publish results.
   *
   * This thread is automatically launched by Connector::connectSocket.
   */
  void cmdThread();

  /**
   * \brief Gets the cyclical status data.
   *
   * This thread is automatically launched by Connector::connectSocket.
   */
  void getThread();

  /**
     * \brief Asynchronously connect to a robot and launch the proper Cmd/Get thread.
     *
     * This method will attempt to async_connect to the robot.  If it fails, it will try again.  When it succeeds, it
     * will launch the appropriate Get/Cmd thread.  If this is being called from the Get/Set thread, it must be launched
     * in a separate, detached thread.  It will attempt to join() the get/set thread before relaunching, to give it a
     * chance to return.
     * @param host The ip address
     * @param port The port
     * @param cmd_type Get/Set
     * @return
     */
  bool connectSocket(std::string host, int port, RobotCommand::CommandType cmd_type);

  /**
   * \brief Publish a robot_movement_interface/Result to the command_result topic.
   *
   * The header timestamp will be set to ros::Time::now() before sending.
   *
   * @param command_id id number of the current command
   * @param result_code 0 == OK
   * @param additional_information Any additional message
   */
  void publishRmiResult(int command_id = 0, int result_code = 0, std::string additional_information = "") const;

  /**
   * \brief Publish a robot_movement_interface/Result to the command_result topic.
   *
   * @param result The complete message to send.
   */
  void publishRmiResult(const robot_movement_interface::Result& result) const;

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
  /// Base port of the robot.  This is the port for Cmd messages.  Get messages are port_ + 1.
  int port_;

  /// asio io service.  Owned by Driver.
  boost::asio::io_service& io_service_;

  /// Queue of all rmi_driver::RobotCommands to be sent by Connector::cmdThread().
  std::deque<RobotCommandPtr> command_list_;

  /// Receives the robot_movement_interface/CommandList for this namespace
  ros::Subscriber command_list_sub_;
  /// Publishes the robot_movement_interface/Result for this namespace
  ros::Publisher command_result_pub_;
  /// Publishes the robot_movement_interface::EulerFrame for this namespace
  ros::Publisher tool_frame_pub_;
  /// NodeHandle for this namespace
  ros::NodeHandle nh_;

  // std::queue<std::shared_ptr<robot_movement_interface::Result>> command_result_list_;

  std::thread get_thread_;
  std::thread cmd_thread_;

  /// The last known joint state.  Set by getThead() and aggregated by the Driver.
  sensor_msgs::JointState last_joint_state_;

  /// The last known tool frame.  Published by publishState(), called from Driver.
  robot_movement_interface::EulerFrame last_tool_frame_;

  /// List of joint names for this robot.
  std::vector<std::string> joint_names_;

  /// The CommandRegister that was loaded by the plugin
  CommandRegisterPtr cmd_register_;

  /// "The ClassLoader must not go out scope while you are using the plugin."  Keep it alive.
  CmdRegLoaderPtr cmh_loader_;

  ///\brief Used to read and consume and messages sent after a cancel.
  /// This was happening to me if I paused the PLC, aborted a command, then unpaused the PLC.
  bool flush_socket_cmd_ = false;

  boost::asio::streambuf socket_cmd_flush_buff_;

  /// Will cause commandListCb to exit if it encounters a message it can't match.
  bool abort_on_fail_to_find_ = true;  /// @todo Will all robot types have an ABORT command?

  /// Connector::cmdThread() will clearCommands if it receives an error response or disconnects
  bool clear_commands_on_error_ = true;

  rmi_log::RmiLogger logger_;
};

}  // namespace rmi_driver

#endif /* INCLUDE_CONNECTOR_H_ */
