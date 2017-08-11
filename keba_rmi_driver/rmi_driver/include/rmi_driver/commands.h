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

#include <ros/ros.h>

#include <robot_movement_interface/Command.h>

#include <string>

namespace keba_rmi_driver
{

/**
 * Commands that will be sent to the robot controller.  They are represented by a string with a command, a :, then parameters
 * <command> : <params>
 */
class Command
{
public:

  //Choose which socket to send over.  Currently only Cmd will do anything.  I'm not sure this will remain.
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

  ~Command()
  {
  }

  std::string toString() const
  {
    std::string ret = command_ + " : " + params_ + "\n";
    return ret;
  }

  /**
   * Converts a float vector into a string of values separated by spaces
   *
   * @param floatVec vector of floats
   * @return string of values
   */
  static std::string paramsToString(const std::vector<float> &floatVec);

  virtual std::ostream& dump(std::ostream& o) const
  {
    o << command_ + " : " + params_;
    return o;
  }

  //Getters/setters

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

inline std::ostream& operator<<(std::ostream& o, const Command& cmd)
{
  return cmd.dump(o);
}

/**
 * Used to prepare command and parameter strings.  Provide the constructor with a similar message.
 * Fill out any fields you want checked.  Vectors will be checked for length, strings for equality.
 * Fields that have a length of 0 will be ignored.
 *
 * Extend this class, set up the sample_command_ in the constructor, and override processMsg.
 *
 * You can also create a base CommandHandler with a sample message and a std::function/lambda to process it.
 */
class CommandHandler
{
public:

  typedef std::function<Command(const robot_movement_interface::Command&)> CommandHandlerFunc;

  CommandHandler()
  {
  }

  virtual ~CommandHandler()
  {
  }

  /**
   * Construct a new CommandHandler.  Not used
   *
   * @param cmd_msg The sample command to match while choosing a handler.  Will be stored.
   */
  //CommandHandler(const robot_movement_interface::Command &cmd_msg);
  /**
   * Construct a new CommandHandler that will call a std::function.  Used to quickly create a new handler without having to
   * create a new class.
   *
   * Example:
   *  CommandHandler chtest(cmd, [](const robot_movement_interface::Command& cmd_msg)
   {
   return Command(Command::CommandType::Cmd, cmd_msg.command_type, cmd_msg.pose_type);
   });
   *
   * @param sample_command The sample command to match
   * @param f a std::function/lambda that takes an rmi::Command returns a telnet Command
   */
  CommandHandler(const robot_movement_interface::Command &sample_command, CommandHandlerFunc f);

  virtual void initialize()
  {
  }

  /**
   * Checks if the values specified in the sample_command match those in cmd_msg.
   * Strings are checked for equality.
   * Vectors are checked for length.
   *
   * @param cmd_msg The message received from the command_list topic
   * @return True if it's a match
   */
  bool operator==(const robot_movement_interface::Command &cmd_msg);

  /**
   * Processes a robot_movement_interface::Command.  Override this method in extended classes.
   * The message should have already been checked for relevance.
   * This base version will call the stored std::function.
   *
   * @param cmd_msg The message received from the command_list topic
   * @param telnet_cmd The command to send to the robot
   * @return True if OK
   */
  virtual bool processMsg(const robot_movement_interface::Command &cmd_msg, Command &telnet_cmd)
  {
    if (!process_func_)
    {
      ROS_ERROR_STREAM("Base CommandHandler::processMsg was called but the process function was not set!");
      return false;
    }

    telnet_cmd = process_func_(cmd_msg);
    return telnet_cmd.getCommand().compare("error") != 0;

  }

  void setProcFunc(CommandHandlerFunc &f)
  {
    process_func_ = f;
  }

  const robot_movement_interface::Command& getSampleCommand() const;

  virtual std::ostream& dump(std::ostream& o) const;

protected:
  robot_movement_interface::Command sample_command_;

  CommandHandlerFunc process_func_ = nullptr;
};

inline std::ostream& operator<<(std::ostream& o, const CommandHandler& cmdh)
{
  return cmdh.dump(o);
}

/**
 * This class contains all the registered command handlers.
 */
class CommandRegister
{
public:

  typedef std::vector<std::unique_ptr<CommandHandler>> CommandHandlerPtrVec;
  CommandRegister()
  {
  }

  virtual ~CommandRegister()
  {
  }

  virtual void initialize()
  {
  }

  /**
   * @todo this should be used like the v2 industrial client's joint map to support multiple robots
   *
   * @param joints
   */
  virtual void initialize(const std::vector<std::string> &joints) = 0;

  /**
   * Create commands and put them in the command_handlers_ vector.
   */
  virtual void registerCommands() = 0;

  /**
   *
   * @return a reference to the vector of handlers
   */
  std::vector<std::unique_ptr<CommandHandler>>& handlers()
  {
    return command_handlers_;
  }

protected:
  CommandHandlerPtrVec command_handlers_;
};

} //namespace keba_rmi_driver

#endif /* INCLUDE_COMMANDS_H_ */
