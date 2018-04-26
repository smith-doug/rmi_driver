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

#include "rmi_driver/connector.h"
#include <boost/algorithm/string.hpp>
#include <boost/asio/use_future.hpp>
#include <chrono>
#include <future>
#include <memory>
#include "rmi_driver/rotation_utils.h"
#include "rmi_driver/util.h"

namespace rmi_driver
{
using namespace boost::asio::ip;

Connector::Connector(std::string ns, boost::asio::io_service &io_service, std::string host, int port,
                     StringVec joint_names, CmdRegLoaderPtr cmd_reg_loader, CommandRegisterPtr cmd_register,
                     bool clear_commands_on_error)
  : ns_(ns)
  , io_service_(io_service)
  , socket_cmd_(io_service)
  , socket_get_(io_service)
  , host_(host)
  , port_(port)
  , cmd_register_(cmd_register)
  , nh_(ns)
  , cmd_reg_loader_(cmd_reg_loader)
  , clear_commands_on_error_(clear_commands_on_error)
  , logger_("CONNECTOR", ns)

{
  joint_names_ = joint_names;

  command_result_pub_ = nh_.advertise<robot_movement_interface::Result>("command_result", 30);
  command_list_sub_ = nh_.subscribe("command_list", 1, &Connector::subCB_CommandList, this);

  tool_frame_pub_ = nh_.advertise<robot_movement_interface::EulerFrame>("tool_frame", 30);

  tool_frame_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tool_frame_pose", 30);

  logger_.INFO() << "Created a new Connector";
}

bool Connector::connect()
{
  return connect(host_, port_);
}

void Connector::stop()
{
  std::cout << "Connector::stop() begin\n";

  this->socket_cmd_.shutdown(boost::asio::socket_base::shutdown_type::shutdown_both);
  this->socket_get_.shutdown(boost::asio::socket_base::shutdown_type::shutdown_both);

  this->socket_cmd_.close();
  this->socket_get_.close();

  if (get_thread_.joinable())
    get_thread_.join();

  if (cmd_thread_.joinable())
    cmd_thread_.join();

  cmd_register_.reset();
  std::cout << "Connector::stop() done" << std::endl;
}

void Connector::cancelSocketCmd(int timeout)
{
  // Give the socket a chance to finish.  If an abort is sent, active motion commands may be able to finish naturally
  // and respond.
  if (socket_cmd_mutex_.try_lock_for(std::chrono::milliseconds(timeout)))
  {
    socket_cmd_mutex_.unlock();

    logger_.INFO() << "cancelSocketCmd() lock successful, no need to cancel";
  }
  else  // It failed to respond.
  {
    socket_cmd_.cancel();  // Cancel any async commands on this socket

    flush_socket_cmd_ = true;
    cmdSocketFlusher();  // Launch the flusher to consume any messages that are sent late.

    logger_.INFO() << "cancelSocketCmd() lock failed, canceling";
  }
}

bool Connector::connectSocket(std::string host, int port, RobotCommand::CommandType cmd_type)
{
  int local_port = port;

  tcp::resolver resolver(io_service_);

  tcp::resolver::query query(host, boost::lexical_cast<std::string>(local_port));
  tcp::resolver::iterator endpointIterator = resolver.resolve(query);

  std::thread *thread;
  boost::asio::ip::tcp::socket *sock;

  if (cmd_type == RobotCommand::CommandType::Cmd)
  {
    thread = &cmd_thread_;
    sock = &socket_cmd_;
  }
  else if (cmd_type == RobotCommand::CommandType::Get)
  {
    thread = &get_thread_;
    sock = &socket_get_;
  }

  // Wait until the correct thread exits cleanly
  if (thread->joinable())
    thread->join();

  boost::asio::async_connect(
      *sock, endpointIterator,
      [this, host, local_port, cmd_type](const boost::system::error_code &ec, tcp::resolver::iterator i) {

        std::string con_type = "NOT SET";
        if (cmd_type == RobotCommand::CommandType::Cmd)
          con_type = "Cmd";
        else if (cmd_type == RobotCommand::CommandType::Get)
          con_type = "Get";

        if (ec)  // If it failed, try again
        {
          // clang format loses its mind and reformats the entire function if I send this straight to ERROR()
          std::stringstream ss;
          ss << "Socket(" << con_type << " " << host << ":" << local_port << ") Ec was set " << ec.message();
          logger_.ERROR() << ss.str();

          // Clear the command list if connecting failed
          if (command_list_.size() > 0 && cmd_type == RobotCommand::CommandType::Cmd)
          {
            publishRmiResult(command_list_.front()->getCommandId(), CommandResultCodes::SOCKET_FAILED_TO_CONNECT,
                             "Cmd socket failed to connect, clearing commands");
            clearCommands();
            logger_.ERROR() << "Clearing command list because the socket failed to connect while commands were waiting";
          }

          std::this_thread::sleep_for(std::chrono::seconds(1));
          connectSocket(host, local_port, cmd_type);
        }
        else  // Connected, launch the correct thread
        {
          if (cmd_type == RobotCommand::CommandType::Cmd)
          {
            cmd_thread_ = std::thread(&Connector::cmdThread, this);
          }
          else if (cmd_type == RobotCommand::CommandType::Get)
          {
            get_thread_ = std::thread(&Connector::getThread, this);
          }

          logger_.INFO() << " Async Socket(" << con_type << ") established to " << host << ":" << local_port;
        }

      });

  return true;
}

bool Connector::connect(std::string host, int port)
{
  tcp::resolver resolver(io_service_);

  tcp::resolver::query query(host, boost::lexical_cast<std::string>(port));
  tcp::resolver::iterator endpointIterator = resolver.resolve(query);

  connectSocket(host, port, RobotCommand::CommandType::Cmd);
  connectSocket(host, port + 1, RobotCommand::CommandType::Get);

  return true;
}

void Connector::cmdSocketFlusher()
{
  if (flush_socket_cmd_)
  {
    // Consume the buffer to reset it.
    socket_cmd_flush_buff_.consume(socket_cmd_flush_buff_.size());

    boost::asio::async_read_until(
        socket_cmd_, socket_cmd_flush_buff_, '\n', [&](const boost::system::error_code &e, std::size_t size) {

          if (e)  // If there is an error code, this was either cancelled or disconnected.
          {
            logger_.INFO() << "Connector::cmdSocketFlusher() is exiting with ec: " << e.message();
            flush_socket_cmd_ = false;
            return;
          }
          else  // Some message was consumed.  That should hopefully be the only one, but read again anyway.
          {
            std::string line;
            std::istream is(&socket_cmd_flush_buff_);
            std::getline(is, line);

            logger_.INFO() << "Connector::cmdSocketFlusher() flushed a message (" << size << "): " << line;
            if (flush_socket_cmd_)
              cmdSocketFlusher();
          }
        });
  }
}

std::string Connector::sendCommand(const RobotCommand &command)
{
  tcp::socket *socket = NULL;
  std::timed_mutex *mutex = NULL;
  if (command.getType() == RobotCommand::CommandType::Get)
  {
    socket = &socket_get_;
    mutex = &socket_get_mutex_;
  }
  else if (command.getType() == RobotCommand::CommandType::Cmd)
  {
    socket = &socket_cmd_;
    mutex = &socket_cmd_mutex_;

    if (flush_socket_cmd_)  // Flusher active, stop it
    {
      flush_socket_cmd_ = false;
      socket->cancel();
    }
  }

  if (socket == NULL || mutex == NULL)
  {
    return "Error: null socket";
  }

  std::string sendStr = command.toString();

  std::lock_guard<std::timed_mutex> lock(*mutex);

  std::promise<size_t> promise_sendCommand;
  auto future_sendCommand = promise_sendCommand.get_future();

  boost::asio::streambuf buff;

  // Perform an asynchronous write.  This enables canceling.
  // I'm using futures instead of launching things from inside the write handler to maintain a more
  // synchronous style.
  boost::asio::async_write(
      *socket, boost::asio::buffer(sendStr), [&](const boost::system::error_code &e, std::size_t size) {
        if (e)
        {
          // Let the future have the exception
          promise_sendCommand.set_exception(std::make_exception_ptr(boost::system::system_error(e)));
        }
        else
        {
          promise_sendCommand.set_value(size);
        }
      });

  try
  {
    future_sendCommand.wait();
  }
  catch (const std::exception &e)
  {
    throw;
  }

  promise_sendCommand = std::promise<size_t>();
  future_sendCommand = promise_sendCommand.get_future();

  boost::asio::async_read_until(*socket, buff, '\n', [&](const boost::system::error_code &e, std::size_t size) {
    if (e)
    {
      promise_sendCommand.set_exception(std::make_exception_ptr(boost::system::system_error(e)));
    }
    else
    {
      promise_sendCommand.set_value(size);
    }
  });

  // old synchronous methods
  // boost::asio::write(*socket, boost::asio::buffer(sendStr));
  // boost::asio::read_until(*socket, buff, '\n');

  /*
   * This <works> but I don't understand it.  The usual boost::asio::use_future doesn't work.  Giving it an allocator
   * makes it happy, but to be honest I don't really understand how use_future_t actually works and gets back to
   * an async result and exactly what kind of allocator it actually wants.
   *
   * See: https://github.com/chriskohlhoff/asio/issues/112
   *
   */
  // static boost::asio::use_future_t<std::allocator<std::size_t>> use_future;
  // std::future<std::size_t> read_future = boost::asio::async_read_until(*socket, buff, '\n', use_future);
  future_sendCommand.wait();
  try
  {
    future_sendCommand.get();
    std::string line;
    std::istream is(&buff);
    std::getline(is, line);

    return line;
  }
  catch (const std::exception &e)
  {
    // Maybe I should throw this up to whatever thread called send?
    throw;
    // return "error internal";
  }
}

void Connector::addCommand(RobotCommandPtr command)
{
  if (command->getType() == RobotCommand::CommandType::Cmd)
  {
    command_list_mutex_.lock();
    command_list_.push_back(command);
    command_list_mutex_.unlock();
  }
  else
  {
    logger_.ERROR() << "Connector::addCommand invalid command type";
  }
}

void Connector::clearCommands()
{
  command_list_mutex_.lock();
  logger_.INFO() << "Connector::clearCommands clearing " << command_list_.size() << " entries";
  command_list_ = std::deque<RobotCommandPtr>();
  command_list_mutex_.unlock();
}

bool Connector::commandListCb(const robot_movement_interface::CommandList &msg)
{
  auto conn = this;
  auto cmd_register = this->getCommandRegister();

  logger_.INFO() << "Received a new command_list of size: " << msg.commands.size();

  // Temporary vector to hold processed commands in.  This allows me to abort and not add any commands if 1 in the list
  // was bad.
  std::vector<RobotCommandPtr> command_vect;

  if (msg.replace_previous_commands)
    conn->clearCommands();

  for (auto &&msg_cmd : msg.commands)
  {
    // Find the appropriate handler
    auto handler = cmd_register->findHandler(msg_cmd);

    if (handler)
    {
      logger_.INFO() << "Found cmd handler: " << handler->getName();

      // Create a new CommandPtr with the found handler
      auto robot_command_ptr = handler->processMsg(msg_cmd);
      if (!robot_command_ptr)
      {
        logger_.ERROR() << "Connector::commandListCb got a null telnet_command_ptr";
        goto error_abort;
      }

      // Set the RobotCommand's id to the id of the message for feedback later
      robot_command_ptr->setCommandId(msg_cmd.command_id);

      // Standard Cmds get added to the queue
      if (robot_command_ptr->getType() == RobotCommand::CommandType::Cmd)
      {
        command_vect.push_back(robot_command_ptr);
      }
      else  // A Get was received as part of a CommandList.
      {
        logger_.WARN() << "Got a high priority command via a message: " << robot_command_ptr->getCommand();

        // Call cancelSocketCmd with async.  It will block while it tries to acquire the mutex.
        auto fut = std::async(std::launch::async, &Connector::cancelSocketCmd, conn, 50);

        std::string send_response = conn->sendCommand(*robot_command_ptr);
        boost::trim_right(send_response);

        // Publish the result  @todo is this really the right thing to do?  It will publish over the same channel.
        auto abort_res_code = CommandResultCodes::ABORT_OK;
        if (!boost::istarts_with(send_response, "aborted"))
        {
          abort_res_code = CommandResultCodes::ABORT_FAIL;
        }
        publishRmiResult(robot_command_ptr->getCommandId(), abort_res_code, send_response);

        logger_.INFO() << "High priority response: " << send_response;

        fut.wait();
      }
      continue;
    }
    else  // if (!handler)
    {
      logger_.ERROR() << "Failed to find cmd handler for: " << msg_cmd;

      // Send a failure response
      publishRmiResult(msg_cmd.command_id, CommandResultCodes::FAILED_TO_FIND_HANDLER, "Failed to find cmd handler");
      /// @todo If I make an ABORT message mandatory, I could use that to actually make the robot stop.
      if (abort_on_fail_to_find_)
      {
        goto error_abort;
      }
    }
  }

  // We made it here without errors so add all the commands to the list.
  for (auto &&cmd : command_vect)
  {
    this->addCommand(cmd);
  }
  return true;

error_abort:
  logger_.ERROR() << " Not adding any commands from this list and clearing any existing commands!";
  command_vect.clear();
  this->clearCommands();
  return true;
}

void Connector::publishRmiResult(int command_id, int result_code, std::string additional_information) const
{
  robot_movement_interface::Result result;
  result.command_id = command_id;
  result.result_code = result_code;
  result.additional_information = additional_information;
  result.header.stamp = ros::Time::now();

  publishRmiResult(result);
}

void Connector::publishRmiResult(const robot_movement_interface::Result &result) const
{
  command_result_pub_.publish(result);
}

RobotCommandPtr Connector::findGetCommand(const std::string &command_type, const std::string &pose_type)
{
  robot_movement_interface::Command rmi_cmd;
  rmi_cmd.command_type = command_type;
  rmi_cmd.pose_type = pose_type;

  // Find the handler for the given command and pose type
  auto get_handler = cmd_register_->findHandler(rmi_cmd);
  if (!get_handler)
  {
    logger_.ERROR() << "Failed to get handler for " << command_type << " " << pose_type;
    return nullptr;
  }

  // Process the message to make a RobotCommand
  auto robot_cmd = get_handler->processMsg(rmi_cmd);
  if (!robot_cmd)
  {
    logger_.ERROR() << "Failed to get a RobotCommand for " << command_type << " " << pose_type;
  }
  return robot_cmd;
}

void Connector::getThread()
{
  util::setThreadName("get_thr");
  ros::Rate rate(50);

  // Fetch the required RobotCommands from the plugin.
  auto get_joint_position = findGetCommand("GET", "JOINT_POSITION");
  auto get_version = findGetCommand("GET", "VERSION");
  auto get_tool_frame = findGetCommand("GET", "TOOL_FRAME");

  auto get_status = findGetCommand("GET", "STATUS");

  if (!get_joint_position || !get_version || !get_tool_frame)
  {
    /// @todo make a nice link to a section of docs
    logger_.FATAL() << "One of the getThread handlers failed to be found.  Your plugin MUST implement these!";

    return;
  }

  RobotCommandStatus *get_status_ptr = 0;
  if (get_status)
  {
    get_status_ptr = (RobotCommandStatus *)get_status.get();
  }

  std::string response;
  std::vector<double> pos_real;

  // Check the version string
  try
  {
    response = sendCommand(*get_version);
    if (response.compare(cmd_register_->getVersion()) != 0)
    {
      logger_.ERROR() << "WARNING!  The version returned by the robot does NOT match the version of the active "
                      << "command register!  Things may not work!";

      logger_.ERROR() << "Command register version: " << cmd_register_->getVersion() << ", robot version: " << response;
    }
    else
    {
      logger_.INFO() << " Command register version matches the robot: " << response;
    }
  }
  catch (const boost::system::system_error &ex)
  {
    logger_.INFO() << "Connector::getThread exception: " << ex.what();

    if (ex.code() != boost::asio::error::operation_aborted)
    {
      // Relaunch the get socket/thread
      std::thread(&Connector::connectSocket, this, host_, port_ + 1, RobotCommand::CommandType::Get).detach();
      return;
    }
  }

  while (ros::ok())
  {
    try
    {
      // Get the joint positions
      if (get_status)
      {
        response = sendCommand(*get_status);
        if (!get_status->checkResponse(response))
        {
          logger_.ERROR() << "Get status failed to process: " << response;
          continue;
        }
        get_status_ptr->updateData(response);
        pos_real = util::stringToDoubleVec(get_status_ptr->getLastJointState());
      }
      else
      {
        response = sendCommand(*get_joint_position);
        if (!get_joint_position->checkResponse(response))
        {
          logger_.ERROR() << "Failed to check joint position.  This is bad: " << response;
          continue;
        }
        pos_real = util::stringToDoubleVec(response);
      }

      last_joint_state_.header.stamp = ros::Time::now();
      last_joint_state_.name = joint_names_;
      last_joint_state_.position = pos_real;

      if (last_joint_state_.name.size() != last_joint_state_.position.size())
      {
        ROS_ERROR_STREAM_THROTTLE(
            1, ns_ << " ERROR: Connector::getThread number of positions received(" << last_joint_state_.position.size()
                   << ") doesn't match number of configured joints(" << last_joint_state_.name.size() << ")!");
      }

      // Get the tool frame in euler zyx
      if (get_status)
      {
        pos_real = util::stringToDoubleVec(get_status_ptr->getLastTcpFrame());
      }
      else
      {
        response = sendCommand(*get_tool_frame);
        if (!get_tool_frame->checkResponse(response))
        {
          logger_.ERROR() << "Failed to check tool frame.  This is bad: " << response;
          continue;
        }
        pos_real = util::stringToDoubleVec(response);
      }
      if (pos_real.size() != 6)
      {
        logger_.ERROR() << " ERROR: Connector::getThread GET TOOL_FRAME size wrong!  Expected 6, got "
                        << pos_real.size() << ".  Raw msg: " << response;
        continue;
      }

      last_tool_frame_.x = pos_real[0];
      last_tool_frame_.y = pos_real[1];
      last_tool_frame_.z = pos_real[2];
      last_tool_frame_.alpha = pos_real[3];
      last_tool_frame_.beta = pos_real[4];
      last_tool_frame_.gamma = pos_real[5];

      // No need to calculate the Pose every time, but I should save the time
      last_tool_frame_pose_.header.stamp = ros::Time::now();
    }
    catch (const boost::bad_lexical_cast &)
    {
      logger_.ERROR() << " Connector::getThread: Unable to parse Get response: " << response;

      continue;
    }
    catch (const boost::system::system_error &ex)
    {
      logger_.INFO() << " Connector::getThread exception: " << ex.what();

      if (ex.code() != boost::asio::error::operation_aborted)
      {
        // Relaunch the get socket/thread
        std::thread(&Connector::connectSocket, this, host_, port_ + 1, RobotCommand::CommandType::Get).detach();
        return;
      }
    }

    rate.sleep();
  }
}

void Connector::cmdThread()
{
  util::setThreadName("cmd_thr");
  ros::Rate rate(30);
  logger_.INFO() << " Connector::cmdThread() starting";

  RobotCommandPtr cmd;

  // Start the flusher.  There could be some message in the buffer if this thread was just restarted.
  flush_socket_cmd_ = true;
  cmdSocketFlusher();

  // Check for messages to send, send them 1 at a time and wait for a response for each one.  Send a
  // robot_movement_interface::Result for each one.
  while (!ros::isShuttingDown())
  {
    // Separate the command list mutex and the socket mutex.  This makes it possible to add/remove commands even if it's
    // waiting for a response.

    bool should_send = false;

    // Check for a message
    command_list_mutex_.lock();
    if (command_list_.size() > 0)
    {
      should_send = true;

      cmd = command_list_.front();

      std::ostringstream oss;
      oss << *cmd;

      auto cmd_str = oss.str();
      logger_.INFO() << " Connector::cmdThread Cmd (" << cmd_str.length() << "): " << cmd_str;
    }
    command_list_mutex_.unlock();

    if (should_send)
    {
      try
      {
        std::string response = sendCommand(*cmd);

        robot_movement_interface::Result result;
        result.command_id = cmd->getCommandId();
        if (cmd->checkResponse(response))
        {
          logger_.INFO() << " Connector::cmdThread sendCommand OK. Response: " << response << "\n";
          result.result_code = 0;
        }
        else
        {
          /// OK only indicates that the command was received and processed successfully and execution should continue,
          /// not that the actual result was good/true/whatever.  A not-OK response is always a problem.
          // ROS_INFO_STREAM(ns_ << " Connector::cmdThread sendCommand NOT OK. Response: " << response << std::endl);
          logger_.ERROR() << " Connector::cmdThread sendCommand NOT OK. Response: " << response << "\n";

          result.result_code = 1;

          ///@todo think about how this might affect the order of responses.
          // Clear the list if set.
          if (clear_commands_on_error_)
          {
            logger_.ERROR() << " Connector::cmdThread is clearing any remaining commands after receiving an error";
            clearCommands();
          }
        }

        // Command was sent and responded to in some way.  Lock n' pop.
        command_list_mutex_.lock();
        if (command_list_.size() > 0)
          command_list_.pop_front();
        command_list_mutex_.unlock();

        result.additional_information = response;

        result.header.stamp = ros::Time::now();
        command_result_pub_.publish(result);

        cmd.reset();
      }
      catch (const boost::system::system_error &ex)
      {
        logger_.INFO() << " Connector::cmdThread exception: " << ex.what();

        // If the error is cause by anything other than a cancel, reconnect
        if (ex.code() != boost::asio::error::operation_aborted)
        {
          // Removing clearCommands() actually allows the robot to continue if the socket is lost due to a plc restart
          // or whatever.
          // An abort command will still clear it out.  Maybe a timer would be safer?
          /// @todo clearing would be safer

          if (clear_commands_on_error_)
          {
            // eof probably indicates that the server's socket was restarted.  Preserve the list and restart the cmd
            // thread to reconnect.  It will pick it up when it comes back around.
            if (ex.code() == boost::asio::error::eof)
            {
              logger_.INFO() << "Socket has to reconnect.  Not clearing command list.";
            }
            else
            {
              logger_.INFO() << " Connector::cmdThread is clearing any remaining commands due to the socket error";
              clearCommands();
            }
          }

          std::thread(&Connector::connectSocket, this, host_, port_, RobotCommand::CommandType::Cmd).detach();

          return;
        }
      }
    }
    else
    {
      rate.sleep();
    }
  }
}

void Connector::publishState()
{
  // Publish the required YPR pose as-is
  tool_frame_pub_.publish(last_tool_frame_);

  // Publish the reported tcp as a PoseStamped.  This makes it easier to use in other tools like RmiCommander or
  // monitoring.
  last_tool_frame_pose_.header.frame_id = "";  //@todo do something with this.  I've never gotten end effectors to
                                               // actually work so I'm not sure how to monitor the active tcp

  last_tool_frame_pose_.pose.position.x = last_tool_frame_.x;
  last_tool_frame_pose_.pose.position.y = last_tool_frame_.y;
  last_tool_frame_pose_.pose.position.z = last_tool_frame_.z;

  // Change the reported pose's orientation into a quaternion.
  tf2::Matrix3x3 matrix;
  matrix.setEulerYPR(last_tool_frame_.alpha, last_tool_frame_.beta, last_tool_frame_.gamma);

  tf2::Quaternion quat;
  matrix.getRotation(quat);

  last_tool_frame_pose_.pose.orientation.w = quat.w();
  last_tool_frame_pose_.pose.orientation.x = quat.x();
  last_tool_frame_pose_.pose.orientation.y = quat.y();
  last_tool_frame_pose_.pose.orientation.z = quat.z();

  tool_frame_pose_pub_.publish(last_tool_frame_pose_);

  // Publish it as a transform for other ROS stuff that can handle tf
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = last_tool_frame_pose_.header.stamp;
  tf.header.frame_id = "world";
  if (ns_ != "/")
    tf.child_frame_id = ns_ + "_tool_frame_pose";
  else
    tf.child_frame_id = ns_ + "tool_frame_pose";

  tf.transform.translation.x = last_tool_frame_pose_.pose.position.x;
  tf.transform.translation.y = last_tool_frame_pose_.pose.position.y;
  tf.transform.translation.z = last_tool_frame_pose_.pose.position.z;
  tf.transform.rotation = last_tool_frame_pose_.pose.orientation;
  tool_frame_pose_br_.sendTransform(tf);
}

}  // namespace rmi_driver
