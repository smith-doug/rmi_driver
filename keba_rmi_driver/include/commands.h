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

#ifndef INCLUDE_COMMANDS_H_
#define INCLUDE_COMMANDS_H_

#include <robot_movement_interface/Command.h>

#include <string>

namespace keba_rmi_driver
{

class Command
{
public:
  enum CommandType
  {
    Get, Cmd
  };

  Command() :
      command_(""), params_(""), type_(CommandType::Cmd)
  {
  }

  Command(CommandType type, const std::string &command, std::string params = "") :
      type_(type), command_(command), params_(params)
  {

  }

  Command(CommandType type, const std::string &command, const std::vector<float> &floatVec) :
      type_(type), command_(command)
  {
    params_ = Command::paramsToString(floatVec);
  }

  std::string toString() const
  {

    std::string ret = command_ + " : " + params_ + "\n";
    return ret;
  }

  static std::string paramsToString(const std::vector<float> &floatVec);




  const std::string& getParams() const;
  void setParams(const std::string& params);
  CommandType getType() const;
  void setType(CommandType type);
  const std::string& getCommand() const;
  void setCommand(const std::string& command);

protected:
  std::string command_;
  std::string params_;

  CommandType type_;
};



/**
 * Used to prepare command and parameter strings.  Provide the constructor with a similar message.
 * Fill out any fields you want checked.  Vectors will be checked for length, strings for equality.
 * Fields that have a length of 0 will be ignored.
 */
class CommandHandler
{



public:

  typedef std::function<Command(const robot_movement_interface::Command&)> CommandHandlerFunc;

  CommandHandler(const robot_movement_interface::Command &cmd_msg);

  CommandHandler(const robot_movement_interface::Command &cmd_msg, CommandHandlerFunc f);

  bool operator==(const robot_movement_interface::Command &cmd_msg);

  robot_movement_interface::Command sample_command_;

  bool processMsg(const robot_movement_interface::Command &cmd_msg, Command &telnet_cmd)
  {
    if (!process_func_)
      return false;

    telnet_cmd = process_func_(cmd_msg);
    return telnet_cmd.getCommand().compare("error") != 0;

  }

  void setProcFunc(CommandHandlerFunc &f)
  {
    process_func_ = f;
  }

  CommandHandlerFunc process_func_ = nullptr;

};


class CommandRegister
{
public:
  CommandRegister()
  {
  }

  virtual void registerCommands() = 0;

  std::vector<CommandHandler> command_handlers_;
};

} //namespace keba_rmi_driver

#endif /* INCLUDE_COMMANDS_H_ */
