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
 *  Created on: Aug 4, 2017
 *      Author: Doug Smith
 */

#include "keba_rmi_plugin/commands_keba.h"
#include <string>
#include <vector>

namespace keba_rmi_driver
{

/**
 * Creates a dynamic parameter string from a float vector
 * @param velocity vector of a full Keba dynamic
 * @return string containing "dyn : velAxis accAxis decAxis jerkAxis vel acc dec jerk velOri accOri decOri jerkOri"
 */
std::string processKebaDyn(const std::vector<float> &velocity)
{
  //Do additional error checking.  Maybe return a bool?

  std::stringstream oss;
  oss << "dyn : ";
  oss << Command::paramsToString(velocity);

  return oss.str();
}

KebaCommands::KebaCommands() :
    commands_registered_(0)
{
  registerCommands();
}

void KebaCommands::initialize()
{
  num_main_joints_ = 6;
  num_aux_joints_ = 1;

  registerCommands();
}

void KebaCommands::initialize(const std::vector<std::string> &joints)
{
  initialize();
}

void KebaCommands::registerCommands()
{
  if (commands_registered_)
    return;

  command_handlers_.emplace_back(new KebaCommandPtpJoints());
  command_handlers_.emplace_back(new KebaCommandLinQuat());
  command_handlers_.emplace_back(new KebaCommandLinEuler());

  //Sample command for lambda usage
  robot_movement_interface::Command cmd;
  cmd.command_type = "TEST";

  CommandHandler chtest(cmd, [](const robot_movement_interface::Command& cmd_msg)
  {
    return Command(Command::CommandType::Cmd, cmd_msg.command_type, cmd_msg.pose_type);
  });

  command_handlers_.emplace_back(new CommandHandler(std::move(chtest)));

  commands_registered_ = true;

}

KebaCommandPtpJoints::KebaCommandPtpJoints()
{
  robot_movement_interface::Command cmd;
  cmd.command_type = "PTP";
  cmd.pose_type = "JOINTS";
  cmd.pose =
  { 0,1,2,3,4,5,6};
//  cmd.velocity_type = "%";
//  cmd.velocity =
//  { 0,1,2,3,4,5,6};

  sample_command_ = cmd;
}

bool KebaCommandPtpJoints::processMsg(const robot_movement_interface::Command& cmd_msg, Command& telnet_cmd)
{
  std::string command_str = "joint move";
  std::string command_params = "";
  std::ostringstream oss;

  oss << Command::paramsToString(cmd_msg.pose);

  if (cmd_msg.velocity_type.compare("DYN") == 0)
  {
    oss << " ";
    oss << processKebaDyn(cmd_msg.velocity);
  }

  command_params = oss.str();

  telnet_cmd = Command(Command::CommandType::Cmd, command_str, command_params);
  return true;
}

KebaCommandLinQuat::KebaCommandLinQuat()
{
  robot_movement_interface::Command cmd;
  cmd = robot_movement_interface::Command();
  cmd.command_type = "LIN";
  cmd.pose_type = "QUATERNION";
  cmd.pose =
  { 0,1,2,3,4,5,6};
  //cmd.velocity =
  //{ 0};

  sample_command_ = cmd;
}

bool KebaCommandLinQuat::processMsg(const robot_movement_interface::Command& cmd_msg, Command& telnet_cmd)
{
  std::string command_str = "linq move";
  std::string command_params = "";
  std::ostringstream oss;

  auto pose_temp = cmd_msg.pose;

  pose_temp[0] *= 1000.0;
  pose_temp[1] *= 1000.0;
  pose_temp[2] *= 1000.0;

  oss << Command::paramsToString(pose_temp);

  if (cmd_msg.velocity_type.compare("DYN") == 0)
  {
    oss << " ";
    oss << processKebaDyn(cmd_msg.velocity);
  }

  command_params = oss.str();

  telnet_cmd = Command(Command::CommandType::Cmd, command_str, command_params);
  return true;

}

KebaCommandLinEuler::KebaCommandLinEuler()
{
  robot_movement_interface::Command cmd;
  cmd = robot_movement_interface::Command();
  cmd.command_type = "LIN";
  cmd.pose_type = "EULER_INTRINSIC_ZYX";
  cmd.pose =
  { 0,1,2,3,4,5};

  sample_command_ = cmd;
}

bool KebaCommandLinEuler::processMsg(const robot_movement_interface::Command& cmd_msg, Command& telnet_cmd)
{
  std::string command_str = "lin move";
  std::string command_params = "";
  std::ostringstream oss;

  auto pose_temp = cmd_msg.pose;

  pose_temp[0] *= 1000.0;
  pose_temp[1] *= 1000.0;
  pose_temp[2] *= 1000.0;

  oss << Command::paramsToString(pose_temp);

  if (cmd_msg.velocity_type.compare("DYN") == 0)
  {
    oss << " ";
    oss << processKebaDyn(cmd_msg.velocity);
  }

  command_params = oss.str();

  telnet_cmd = Command(Command::CommandType::Cmd, command_str, command_params);
  return true;
}

} //namespace keba_rmi_driver

