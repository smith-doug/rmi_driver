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
 *  Created on: Aug 4, 2017
 *      Author: Doug Smith
 */

#include "keba_rmi_plugin/commands_keba.h"
#include <boost/algorithm/string.hpp>
#include <memory>
#include <string>
#include <vector>
#include "keba_rmi_plugin/keba_util.h"

namespace keba_rmi_plugin
{
KebaCommandRegister::KebaCommandRegister() : commands_registered_(0), num_aux_joints_(0), num_main_joints_(0)
{
  // registerCommands();
}

void KebaCommandRegister::initialize(const std::vector<std::string> &joints)
{
  joint_names_ = joints;

  num_main_joints_ = 6;
  num_aux_joints_ = 1;  //@todo figure this out

  registerCommandHandlers();
}

void KebaCommandRegister::registerCommandHandlers()
{
  if (commands_registered_)
    return;

  // Add the required Connector::getThread handler
  this->addHandler<KebaCommandGet>();

  // Motion commands
  this->addHandler<KebaCommandPtp>();
  this->addHandler<KebaCommandLin>();

  // Settings
  this->addHandler<KebaCommandDyn>();
  this->addHandler<KebaCommandOvl>();

  // Other
  this->addHandler<KebaCommandAbort>();
  this->addHandler<KebaCommandSync>();
  this->addHandler<KebaCommandWait>();

  // Sample command for lambda usage
  robot_movement_interface::Command cmd;
  cmd.command_type = "TEST";

  auto chtest = CommandHandler::createHandler(cmd, [](const robot_movement_interface::Command &cmd_msg) {
    return std::make_shared<RobotCommand>(RobotCommand::CommandType::Cmd, cmd_msg.command_type, cmd_msg.pose_type);
  });

  this->addHandler(std::move(chtest));

  commands_registered_ = true;
}

KebaCommandGet::KebaCommandGet()
{
  handler_name_ = "KebaCommandGet";

  robot_movement_interface::Command cmd;
  cmd = robot_movement_interface::Command();
  cmd.command_type = "GET";
  cmd.pose_type = "JOINT_POSITION|TOOL_FRAME|VERSION";

  sample_command_ = cmd;
}

RobotCommandPtr KebaCommandGet::processMsg(const robot_movement_interface::Command &cmd_msg) const
{
  std::string cmd_str = "get ";
  RobotCommandPtr cmd_ptr = std::make_shared<KebaCommand>(RobotCommand::RobotCommand::CommandType::Get);
  if (boost::iequals("JOINT_POSITION", cmd_msg.pose_type))
    cmd_str += "joint position";
  else if (boost::iequals("TOOL_FRAME", cmd_msg.pose_type))
    cmd_str += "tool frame";
  else if (boost::iequals("VERSION", cmd_msg.pose_type))
    cmd_str += "version";
  else
    return nullptr;

  cmd_ptr->setCommand(cmd_str, "");
  return cmd_ptr;
}

KebaCommandLin::KebaCommandLin()
{
}

void KebaCommandLin::initialize()
{
  handler_name_ = "KebaCommandLin";

  robot_movement_interface::Command cmd;
  cmd = robot_movement_interface::Command();
  cmd.command_type = "LIN";
  cmd.pose_type = "QUATERNION|EULER_INTRINSIC_ZYX|JOINTS";

  sample_command_ = cmd;
  auto num_joints = getCommandRegister()->joint_names_.size();
  sample_command_.pose = { 7, 6, (float)num_joints };
}

RobotCommandPtr KebaCommandLin::processMsg(const robot_movement_interface::Command &cmd_msg) const
{
  RobotCommandPtr cmd_ptr = std::make_shared<KebaCommand>(RobotCommand::RobotCommand::CommandType::Cmd);

  std::string command_str = "lin " + boost::to_lower_copy(cmd_msg.pose_type);

  auto pose_temp = cmd_msg.pose;

  auto cmd_register = this->getCommandRegister();

  if (cmd_msg.pose_type.compare("JOINTS") == 0)
  {
    // Check if the pose size is smaller than the number of joints
    // This should probably be if != but I have too many test bags that have more joints
    if (cmd_msg.pose.size() < cmd_register->joint_names_.size())
    {
      ROS_ERROR_STREAM("Invalid KebaCommandLin message: cmd_msg.pose.size() " << cmd_msg.pose.size()
                                                                              << " < cmd_register->joint_names_.size() "
                                                                              << cmd_register->joint_names_.size());
      return nullptr;
    }
  }
  else if (cmd_msg.pose_type.compare("QUATERNION") == 0 || cmd_msg.pose_type.compare("EULER_INTRINSIC_ZYX"))
  {
    pose_temp[0] *= 1000.0;
    pose_temp[1] *= 1000.0;
    pose_temp[2] *= 1000.0;

    try  // Check for aux values
    {
      processKebaAux(cmd_msg, *cmd_ptr);
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM("KebaCommandLin::processMsg processKebaAux failed: " << ex.what());
      return nullptr;
    }
  }

  cmd_ptr->setCommand(command_str, RobotCommand::paramsToString(pose_temp));

  // ROS joint isn't enough for a Lin.  Only look for a DYN.
  try
  {
    bool had_a_keba_dyn = processKebaDyn(cmd_msg, *cmd_ptr);
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM("KebaCommandLin::processMsg processKebaDyn failed: " << ex.what());
    return nullptr;
  }

  return cmd_ptr;
}

KebaCommandPtp::KebaCommandPtp()
{
  handler_name_ = "KebaCommandPtp";

  robot_movement_interface::Command cmd;
  cmd.command_type = "PTP";
  cmd.pose_type = "QUATERNION|EULER_INTRINSIC_ZYX|JOINTS";

  sample_command_ = cmd;
}

void KebaCommandPtp::initialize()
{
  auto num_joints = getCommandRegister()->joint_names_.size();
  sample_command_.pose = { 7, 6, (float)num_joints };
}

RobotCommandPtr KebaCommandPtp::processMsg(const robot_movement_interface::Command &cmd_msg) const
{
  std::cout << "reg size: " << getCommandRegister()->joint_names_.size() << std::endl;
  RobotCommandPtr cmd_ptr = std::make_shared<KebaCommand>(RobotCommand::RobotCommand::CommandType::Cmd);

  std::string command_str = "ptp " + boost::to_lower_copy(cmd_msg.pose_type);

  auto pose_temp = cmd_msg.pose;

  if (cmd_msg.pose_type.compare("JOINTS") == 0)
  {
    //@todo check length and stuff
  }
  else if (cmd_msg.pose_type.compare("QUATERNION") == 0 || cmd_msg.pose_type.compare("EULER_INTRINSIC_ZYX"))
  {
    pose_temp[0] *= 1000.0;
    pose_temp[1] *= 1000.0;
    pose_temp[2] *= 1000.0;
    processKebaAux(cmd_msg, *cmd_ptr);
  }

  cmd_ptr->setCommand(command_str, RobotCommand::paramsToString(pose_temp));

  try
  {
    bool had_a_keba_dyn = processKebaDyn(cmd_msg, *cmd_ptr);
    if (!had_a_keba_dyn)
    {
      // Joint velo/accel is good enough for a PTP move
      processRosDyn(cmd_msg, *cmd_ptr);
    }
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM("KebaCommandPtp::processMsg processKebaDyn failed: " << ex.what());
    return nullptr;
  }

  return cmd_ptr;
}

KebaCommandDyn::KebaCommandDyn()
{
  handler_name_ = "KebaCommandDyn";

  robot_movement_interface::Command cmd;
  cmd.command_type = "SETTING";
  cmd.pose_type = "DYN";
  cmd.velocity_type = "DYN";

  sample_command_ = cmd;
}

RobotCommandPtr KebaCommandDyn::processMsg(const robot_movement_interface::Command &cmd_msg) const
{
  RobotCommandPtr cmd_ptr = std::make_shared<KebaCommand>(RobotCommand::RobotCommand::CommandType::Cmd);
  std::string command_str = "setting dyn";
  if (cmd_msg.velocity_type.compare("DYN") == 0)
  {
    processKebaDyn(cmd_msg, *cmd_ptr);
  }

  cmd_ptr->setCommand(command_str, "");

  return cmd_ptr;
}

KebaCommandOvl::KebaCommandOvl()
{
  handler_name_ = "KebaCommandOvl";

  robot_movement_interface::Command cmd;
  cmd.command_type = "SETTING";
  cmd.pose_type = "OVL";
  cmd.blending_type = "%|OVLABS|OVLREL|OVLSUPPOS";

  sample_command_ = cmd;
}

RobotCommandPtr KebaCommandOvl::processMsg(const robot_movement_interface::Command &cmd_msg) const
{
  RobotCommandPtr cmd_ptr = std::make_shared<KebaCommand>(RobotCommand::RobotCommand::CommandType::Cmd);
  std::string command_str = "setting ovl";

  if (processKebaOvl(cmd_msg, *cmd_ptr))
  {
    cmd_ptr->setCommand(command_str, "");
    return cmd_ptr;
  }
  else
  {
    return nullptr;
  }
}

KebaCommandAbort::KebaCommandAbort()
{
  handler_name_ = "KebaCommandAbort";

  robot_movement_interface::Command cmd;
  cmd.command_type = "ABORT";
  sample_command_ = cmd;
}

RobotCommandPtr KebaCommandAbort::processMsg(const robot_movement_interface::Command &cmd_msg) const
{
  RobotCommandPtr cmd_ptr = std::make_shared<KebaCommand>(RobotCommand::RobotCommand::CommandType::Get);
  cmd_ptr->setCommand("abort", "");
  return cmd_ptr;
}

KebaCommandSync::KebaCommandSync()
{
  handler_name_ = "KebaCommandSync";

  robot_movement_interface::Command cmd;
  cmd.command_type = "SYNC";
  cmd.pose_type = "SYNC_NUM";
  cmd.pose = { 1 };
  sample_command_ = cmd;
}

RobotCommandPtr KebaCommandSync::processMsg(const robot_movement_interface::Command &cmd_msg) const
{
  RobotCommandPtr cmd_ptr = std::make_shared<KebaCommand>(RobotCommand::RobotCommand::CommandType::Cmd);

  cmd_ptr->setCommand("sync", RobotCommand::paramsToString(cmd_msg.pose));
  return cmd_ptr;
}

KebaCommandWait::KebaCommandWait()
{
  handler_name_ = "KebaCommandWait";

  robot_movement_interface::Command cmd;
  cmd.command_type = "WAIT";
  cmd.pose_type = "IS_FINISHED";  // Can add more types later
  sample_command_ = cmd;
}

RobotCommandPtr KebaCommandWait::processMsg(const robot_movement_interface::Command &cmd_msg) const
{
  RobotCommandPtr cmd_ptr = std::make_shared<KebaCommand>(RobotCommand::RobotCommand::CommandType::Cmd);

  std::string command_str = "wait " + boost::to_lower_copy(cmd_msg.pose_type);
  cmd_ptr->setCommand(command_str, "");

  return cmd_ptr;
}

}  // namespace keba_rmi_plugin
