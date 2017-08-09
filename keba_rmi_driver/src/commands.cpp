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
 *  Created on: Aug 3, 2017
 *      Author: Doug Smith
 */

#include "commands.h"
#include <ros/ros.h>
#include <robot_movement_interface/Command.h>
#include <vector>

namespace keba_rmi_driver
{

std::string Command::paramsToString(const std::vector<float>& floatVec)
{
  if (floatVec.empty())
    return "";

  std::ostringstream oss;
  std::copy(floatVec.begin(), floatVec.end() - 1, std::ostream_iterator<double>(oss, " "));
  oss << floatVec.back();

  return oss.str();
}

CommandHandler::CommandHandler(const robot_movement_interface::Command& cmd_msg) :
    sample_command_(cmd_msg)
{
}

CommandHandler::CommandHandler(const robot_movement_interface::Command& cmd_msg, CommandHandlerFunc f) :
    sample_command_(cmd_msg), process_func_(std::move(f))
{
}

//Returns True if the sample length is > 0 and it doesn't match msg.
//If sample is "" it will return false as this param isn't used in the match.
bool usedAndNotEqual(const std::string &sample, const std::string &msg)
{
  return sample.length() > 0 && sample.compare(msg) != 0;
}

bool usedAndNotEqual(const std::vector<float> &sample, const std::vector<float> &msg)
{
  return sample.size() > 0 && sample.size() != msg.size();
}

bool CommandHandler::operator ==(const robot_movement_interface::Command &cmd_msg)
{
  //Check strings for usage and equality
  //Check vectors for usage and length
  if (usedAndNotEqual(sample_command_.command_type, cmd_msg.command_type))
    return false;

  if (usedAndNotEqual(sample_command_.pose_reference, cmd_msg.pose_reference))
    return false;

  if (usedAndNotEqual(sample_command_.pose_type, cmd_msg.pose_type))
    return false;

  if (usedAndNotEqual(sample_command_.pose, cmd_msg.pose))
    return false;

  if (usedAndNotEqual(sample_command_.velocity_type, cmd_msg.velocity_type))
    return false;

  return true; //If it got this far it's a match

}

const std::string& Command::getCommand() const
{
  return command_;
}

void Command::setCommand(const std::string& command)
{
  command_ = command;
}

const std::string& Command::getParams() const
{
  return params_;
}

void Command::setParams(const std::string& params)
{
  params_ = params;
}

Command::CommandType Command::getType() const
{
  return type_;
}

void Command::setType(CommandType type)
{
  type_ = type;
}

} //namespace keba_rmi_driver

