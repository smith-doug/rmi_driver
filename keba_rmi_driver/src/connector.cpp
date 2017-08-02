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

#include "connector.h"

namespace keba_rmi_driver
{

using namespace boost::asio::ip;

Connector::Connector(boost::asio::io_service& io_service, std::string host, int port) :
    io_service_(io_service), socket_cmd_(io_service), socket_get_(io_service), host_(host), port_(port)
{

}

bool Connector::connect()
{
  return connect(host_, port_);
}

bool Connector::connect(std::string host, int port)
{
  tcp::resolver resolver(io_service_);

  tcp::resolver::query query(host, boost::lexical_cast<std::string>(port));
  tcp::resolver::iterator endpointIterator = resolver.resolve(query);

  socket_cmd_.connect(*endpointIterator);
  ROS_INFO_NAMED("connector", "cmd connection established to %s:%i", host.c_str(), port);

  port++;
  query = tcp::resolver::query(host, boost::lexical_cast<std::string>(port));
  endpointIterator = resolver.resolve(query);
  socket_get_.connect(*endpointIterator);
  ROS_INFO_NAMED("connector", "get connection established to %s:%i", host.c_str(), port);


  get_thread_ = std::thread(&Connector::getThread, this);
  cmd_thread_ = std::thread(&Connector::cmdThread, this);


  Command cmd(Command::CommandType::Cmd, "joint move", "0.1 0.2 0.3 0.4 0.5 0.6 0.7");

  addCommand(cmd);
  cmd = Command(Command::CommandType::Cmd, "joint move", "0.5 0.2 0.3 0.4 0.5 0.6 0.7");
  addCommand(cmd);
}

std::string Connector::sendCommand(const Command &command)
{

  tcp::socket *socket = 0;
  if(command.type_ == Command::CommandType::Get)
  {
    socket = &socket_get_;
  }
  else if(command.type_ == Command::CommandType::Cmd)
  {
    socket = &socket_cmd_;
  }

  if(socket == NULL)
  {
    return "Error: null socket";
  }

  std::string sendStr = command.toString();
  boost::asio::write(*socket, boost::asio::buffer(sendStr));

  boost::asio::streambuf buff;


  boost::asio::read_until(*socket, buff, '\n');

  std::string line;
  std::istream is(&buff);
  std::getline(is, line);
  //std::istream is(&buff);
  //std::string ret;
  //is >> ret;
  return line;
}

void Connector::addCommand(const Command &command)
{
  if(command.type_ == Command::CommandType::Cmd)
  {
    socket_cmd_mutex_.lock();
    command_list_.push(command);
    socket_cmd_mutex_.unlock();
  }
}

void Connector::getThread()
{
  ros::Rate rate(1);

  while (!ros::isShuttingDown())
  {
    Command cmd(Command::CommandType::Get, "get joint position", "");
    std::string response = sendCommand(cmd);
    std::cout << "Response: " << response << std::endl;


    cmd = Command(Command::CommandType::Get, "get tool frame");

    response = sendCommand(cmd);
    std::cout << "Response: " << response << std::endl;


    rate.sleep();
  }
}

void Connector::cmdThread()
{
  ros::Rate rate(10);
  std::cout << "Cmd starting" << std::endl;

  while (!ros::isShuttingDown())
  {
    if(command_list_.size() > 0)
    {

      socket_cmd_mutex_.lock();
      Command cmd = command_list_.front();
      command_list_.pop();
      std::cout << "Cmd : " << cmd.toString() << std::endl;
      socket_cmd_mutex_.unlock();

      std::string response = sendCommand(cmd);

      std::cout << "Cmd response: " << response << std::endl;
    }
    else
    {
      rate.sleep();
    }
  }
}

} //namespace keba_rmi_driver

