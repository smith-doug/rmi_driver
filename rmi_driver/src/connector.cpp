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
#include "rmi_driver/util.h"

namespace rmi_driver
{
using namespace boost::asio::ip;

Connector::Connector(std::string ns, boost::asio::io_service &io_service, std::string host, int port,
                     StringVec joint_names, CommandRegisterPtr cmd_register, CmhLoaderPtr cmh_loader,
                     bool clear_commands_on_error)
  : ns_(ns)
  , io_service_(io_service)
  , socket_cmd_(io_service)
  , socket_get_(io_service)
  , host_(host)
  , port_(port)
  , cmd_register_(cmd_register)
  , nh_(ns)
  , cmh_loader_(cmh_loader)
  , clear_commands_on_error_(clear_commands_on_error)
  , logger_("CONNECTOR", ns)

{
  joint_names_ = joint_names;

  command_result_pub_ = nh_.advertise<robot_movement_interface::Result>("command_result", 30);
  command_list_sub_ = nh_.subscribe("command_list", 1, &Connector::subCB_CommandList, this);

  tool_frame_pub_ = nh_.advertise<robot_movement_interface::EulerFrame>("tool_frame", 30);

  logger_.INFO() << "Created a new Connector";
}

bool Connector::connect()
{
  return connect(host_, port_);
}

// bool Connector::connectCmd(std::string host, int port)
//{
//  int local_port = port;
//  tcp::resolver resolver(io_service_);
//
//  tcp::resolver::query query(host, boost::lexical_cast<std::string>(local_port));
//  tcp::resolver::iterator endpointIterator = resolver.resolve(query);
//
//  if (cmd_thread_.joinable())
//    cmd_thread_.join();
//
//  boost::asio::async_connect(socket_cmd_, endpointIterator,
//                             [this, host, local_port](const boost::system::error_code &ec, tcp::resolver::iterator i)
//                             {
//                               if (ec)
//                               {
//                                 ROS_INFO_STREAM("Ec was set " << ec.message());
//                                 std::this_thread::sleep_for(std::chrono::seconds(1));
//                                 connectCmd(host, local_port);
//                               }
//                               else
//                               {
//                                 ROS_INFO_STREAM("Async cmd establisted to " << host << ":" << local_port);
//
//                                 cmd_thread_ = std::thread(&Connector::cmdThread, this);
//                               }
//
//                             });
//
//  return true;
//}

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
          logger_.ERROR() << "Socket(" << con_type << ") Ec was set " << ec.message();

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

  // socket_cmd_.connect(*endpointIterator);
  // ROS_INFO_NAMED("connector", "cmd connection established to %s:%i", host.c_str(), port);
  // cmd_thread_ = std::thread(&Connector::cmdThread, this);

  // connectCmd(host, port);
  connectSocket(host, port, RobotCommand::CommandType::Cmd);
  connectSocket(host, port + 1, RobotCommand::CommandType::Get);

  //  query = tcp::resolver::query(host, boost::lexical_cast<std::string>(port + 1));
  //  endpointIterator = resolver.resolve(query);
  //  socket_get_.connect(*endpointIterator);
  //  ROS_INFO_NAMED("connector", "get connection established to %s:%i", host.c_str(), port + 1);
  //
  //  get_thread_ = std::thread(&Connector::getThread, this);

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

RobotCommandPtr Connector::findGetCommandHandler(const std::string &command_type, const std::string &pose_type)
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
  auto get_joint_position = findGetCommandHandler("GET", "JOINT_POSITION");
  auto get_version = findGetCommandHandler("GET", "VERSION");
  auto get_tool_frame = findGetCommandHandler("GET", "TOOL_FRAME");

  if (!get_joint_position || !get_version || !get_tool_frame)
  {
    /// @todo make a nice link to a section of docs
    logger_.FATAL() << "One of the getThread handlers failed to be found.  Your plugin MUST implement these!";

    return;
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
      response = sendCommand(*get_joint_position);
      pos_real = util::stringToDoubleVec(response);

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
      response = sendCommand(*get_tool_frame);
      pos_real = util::stringToDoubleVec(response);
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
          logger_.INFO() << " Connector::cmdThread sendCommand NOT OK. Response: " << response << "\n";

          result.result_code = 1;

          ///@todo think about how this might affect the order of responses.
          // Clear the list if set.
          if (clear_commands_on_error_)
          {
            logger_.INFO() << " Connector::cmdThread is clearing any remaining commands after receiving an error";
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
  tool_frame_pub_.publish(last_tool_frame_);
}

}  // namespace rmi_driver
