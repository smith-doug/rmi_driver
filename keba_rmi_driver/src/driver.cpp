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

#include "driver.h"
#include <iostream>

namespace keba_rmi_driver
{
Driver::Driver()
{
}

void Driver::start()
{

  this->addConnection("192.168.100.100", 30000);

  joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  command_list_sub_ = nh.subscribe("command_list", 1, &Driver::subCB_CommandList, this);

  pub_thread_ = std::thread(&Driver::publishJointState, this);

}

std::string paramsToString(const std::vector<float> &floatVec)
{
  if (floatVec.empty())
    return "";

  std::ostringstream oss;
  std::copy(floatVec.begin(), floatVec.end() - 1, std::ostream_iterator<double>(oss, " "));
  oss << floatVec.back();

  return oss.str();
}

bool Driver::commandListCb(const robot_movement_interface::CommandList &msg)
{

  std::cout << "Got a command" << std::endl;

  auto &conn = conn_map_.begin()->second;

  if (conn_map_.begin() == conn_map_.end() || conn_map_.begin()->second == NULL)
    return false;

  for (auto &msg_cmd : msg.commands)
  {
    std::string command_str = "";
    std::string command_params = "";

    std::ostringstream oss;

    if (msg_cmd.command_type.compare("PTP") == 0)
    {
      if (msg_cmd.pose_type.compare("JOINTS") == 0)
      {
        command_str = "joint move";

        if (msg_cmd.pose.size() != 7)
          return false;

        std::copy(msg_cmd.pose.begin(), msg_cmd.pose.end() - 1, std::ostream_iterator<double>(oss, " "));
        oss << msg_cmd.pose.back();

        command_params = oss.str();
      }
    }

    else if (msg_cmd.command_type.compare("LIN") == 0)
    {
      auto pose_temp = msg_cmd.pose;

      if (msg_cmd.pose_type.compare("QUATERNION") == 0)
      {
        if (msg_cmd.pose.size() != 7)
          return false;

        command_str = "linq move";

      }
      else if (msg_cmd.pose_type.compare("EULER_INTRINSIC_ZYX") == 0)
      {
        if (msg_cmd.pose.size() != 6)
          return false;

        command_str = "lin move";

      }

      pose_temp[0] *= 1000.0;
      pose_temp[1] *= 1000.0;
      pose_temp[2] *= 1000.0;

      command_params = paramsToString(pose_temp);
    }

    if (command_str != "")
    {
      Command telnet_command(Command::CommandType::Cmd, command_str, command_params);
      conn->addCommand(telnet_command);
    }

  }

  return true;

}

void Driver::addConnection(std::string host, int port)
{
  conn_num_++;

  std::vector<std::string> joint_names {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                        "wrist_2_joint", "wrist_3_joint", "rail_to_base"};

  auto shared = std::make_shared<Connector>(io_service_, host, port, joint_names);
  conn_map_.emplace(conn_num_, shared);

  auto &conn = conn_map_.at(conn_num_);
  conn->connect();
  //conn.connect(host, port);
}

void Driver::publishJointState()
{
  ros::Rate pub_rate(30);
  std::cout << "Driver pub" << std::endl;
  ROS_INFO_NAMED("Driver", "publishJointState");
  while (ros::ok())
  {
    for (auto &conn : conn_map_)
    {
      auto lastState = conn.second->getLastJointState();
      joint_state_publisher_.publish(lastState);
    }
    pub_rate.sleep();
  }
}

} // namespace keba_rmi_driver

