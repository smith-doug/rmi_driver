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

#include "rmi_driver/connector.h"
#include <boost/algorithm/string.hpp>
#include <future>
#include <memory>
#include <boost/asio/use_future.hpp>

namespace rmi_driver
{

using namespace boost::asio::ip;

template<typename Out>
void split(const std::string &s, char delim, Out result)
{
  std::stringstream ss;
  ss.str(s);
  std::string item;
  while (std::getline(ss, item, delim))
  {
    std::stringstream temp_ss(item);
    *(result++) = item;
  }
}

std::vector<std::string> split(const std::string &s, char delim)
{
  std::vector<std::string> elems;
  split(s, delim, std::back_inserter(elems));
  return elems;
}

std::vector<double> stringToDoubleVec(const std::string &s)
{
  std::vector<std::string> strVec = split(s, ' ');
  std::vector<double> doubleVec;

  std::transform(strVec.begin(), strVec.end(), std::back_inserter(doubleVec), [](const std::string& val)
  {
    return boost::lexical_cast<double>(val);
  });

  return doubleVec;

}

Connector::Connector(boost::asio::io_service& io_service, std::string host, int port, StringVec joint_names,
                     CommandRegisterPtr cmd_register) :
    io_service_(io_service), socket_cmd_(io_service), socket_get_(io_service), host_(host), port_(port), cmd_register_(
        cmd_register)
{
  joint_names_ = joint_names;

}

bool Connector::connect()
{
  return connect(host_, port_);
}

bool Connector::connectCmd(std::string host, int port)
{
  int local_port = port;
  tcp::resolver resolver(io_service_);

  tcp::resolver::query query(host, boost::lexical_cast<std::string>(local_port));
  tcp::resolver::iterator endpointIterator = resolver.resolve(query);

  boost::asio::async_connect(
      socket_cmd_, endpointIterator, [=](const boost::system::error_code& ec, tcp::resolver::iterator i)
      {
        ROS_INFO_NAMED("connector", "cmd fafssadconnection established to %s:%i", host.c_str(), local_port);
        //this->cmdThread();

      //auto handle = std::async(std::launch::async, &Connector::cmdThread, this);

      //handle.wait();
      cmd_thread_ = std::thread(&Connector::cmdThread, this);

//      cmd_thread_.join();
//      if(ros::ok)
//      {
//        connectCmd(host, local_port);
//      }
    });

  return true;
}

bool Connector::connect(std::string host, int port)
{
  tcp::resolver resolver(io_service_);

  tcp::resolver::query query(host, boost::lexical_cast<std::string>(port + 1));
  tcp::resolver::iterator endpointIterator = resolver.resolve(query);

  //socket_cmd_.connect(*endpointIterator);

  connectCmd(host, port);

  ROS_INFO_NAMED("connector", "cmd connection established to %s:%i", host.c_str(), port);

  port++;
  query = tcp::resolver::query(host, boost::lexical_cast<std::string>(port));
  endpointIterator = resolver.resolve(query);
  socket_get_.connect(*endpointIterator);
  ROS_INFO_NAMED("connector", "get connection established to %s:%i", host.c_str(), port);

  get_thread_ = std::thread(&Connector::getThread, this);
  //cmd_thread_ = std::thread(&Connector::cmdThread, this);

  //io_service_.run();

  return true;
  /*
   Command cmd(Command::CommandType::Cmd, "joint move", "0.1 0.2 0.3 0.4 0.5 0.6 0.7");

   addCommand(cmd);
   cmd = Command(Command::CommandType::Cmd, "joint move", "0.5 0.2 0.3 0.4 0.5 0.6 0.7");
   addCommand(cmd); */
}

std::string Connector::sendCommand(const Command &command)
{

  tcp::socket *socket = NULL;
  std::mutex *mutex = NULL;
  if (command.getType() == Command::CommandType::Get)
  {
    socket = &socket_get_;
    mutex = &socket_get_mutex_;
  }
  else if (command.getType() == Command::CommandType::Cmd)
  {
    socket = &socket_cmd_;
    mutex = &socket_cmd_mutex_;
  }

  if (socket == NULL || mutex == NULL)
  {
    return "Error: null socket";
  }

  std::string sendStr = command.toString();

  std::lock_guard<std::mutex> lock(*mutex);

  std::promise<size_t> promise_write;
  auto write_future = promise_write.get_future();

  boost::asio::async_write(*socket, boost::asio::buffer(sendStr),
                           [&](const boost::system::error_code& e, std::size_t size)
                           {
                             if(e)
                             {
                               promise_write.set_exception(std::make_exception_ptr(boost::system::system_error(e)));
                             }
                             else
                             {
                               promise_write.set_value(size);
                             }

                           });

  write_future.wait();

  //boost::asio::write(*socket, boost::asio::buffer(sendStr));
  boost::asio::streambuf buff;

  //boost::asio::read_until(*socket, buff, '\n');

  //Perform an asynchronous read.
  std::promise<size_t> promise_read;
  auto read_future = promise_read.get_future();

  boost::asio::async_read_until(*socket, buff, '\n', [&](const boost::system::error_code& e, std::size_t size)
  {
    if(e)
    {
      promise_read.set_exception(std::make_exception_ptr(boost::system::system_error(e)));
    }
    else
    {
      promise_read.set_value(size);
    }
  });

  /*
   * This <works> but I don't understand it.  The usual boost::asio::use_future doesn't work.  Giving it an allocator
   * makes it happy, but to be honest I don't really understand how use_future_t actually works and gets back to
   * an async result and exactly what kind of allocator is actually wants.
   *
   * See: https://github.com/chriskohlhoff/asio/issues/112
   *
   */
  //static boost::asio::use_future_t<std::allocator<std::size_t>> use_future;
  //std::future<std::size_t> read_future = boost::asio::async_read_until(*socket, buff, '\n', use_future);

  read_future.wait();
  try
  {
    read_future.get();
    std::string line;
    std::istream is(&buff);
    std::getline(is, line);

    return line;
  }
  catch (const std::exception &e)
  {
    //Maybe I should throw this up to whatever thread called send?
    return "error internal";
  }

}

void Connector::addCommand(const Command &command)
{
  if (command.getType() == Command::CommandType::Cmd)
  {
    command_list_mutex_.lock();
    command_list_.push(command);
    command_list_mutex_.unlock();
  }
}

void Connector::clearCommands()
{
  command_list_mutex_.lock();
  command_list_ = std::queue<Command>();
  command_list_mutex_.unlock();
}

void Connector::getThread()
{
  ros::Rate rate(50);

  //io_service_.run();
  //local_io_service_.run();
  std::cout << "io service running\n";

  Command get_joint_position(Command::CommandType::Get, "get joint position");
  Command get_tool_frame(Command::CommandType::Get, "get tool frame");

  while (ros::ok())
  {
    std::string response = sendCommand(get_joint_position);

    std::vector<double> pos_real;
    try
    {
      pos_real = stringToDoubleVec(response);
    }
    catch (const boost::bad_lexical_cast &)
    {
      ROS_ERROR_STREAM("Unable to parse joint positions");
      continue;
    }

    response = sendCommand(get_tool_frame);

    last_joint_state_.header.stamp = ros::Time::now();
    last_joint_state_.name = joint_names_;
    last_joint_state_.position = pos_real;

    rate.sleep();
  }
}

void Connector::cmdThread()
{
  ros::Rate rate(30);
  std::cout << "Cmd starting" << std::endl;

  Command cmd;

  while (!ros::isShuttingDown())
  {
    //Separate the command list mutex and the socket mutex.  This makes it possible to add/remove commands even if it's
    //waiting for a response.

    bool should_send = false;
    command_list_mutex_.lock();
    if (command_list_.size() > 0)
    {
      should_send = true;

      cmd = command_list_.front();
      command_list_.pop();

      std::ostringstream oss;
      oss << cmd;

      ROS_INFO_STREAM("Cmd (" << oss.str().length() << "): " << cmd);
    }
    command_list_mutex_.unlock();

    if (should_send)
    {
      std::string response = sendCommand(cmd);
      ROS_INFO_STREAM("Cmd response: " << response << std::endl);
    }
    else
    {
      rate.sleep();
    }
  }
}

} //namespace rmi_driver

