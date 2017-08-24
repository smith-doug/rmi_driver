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

namespace rmi_driver
{
/**
 * \brief Commands that will be sent to the robot controller as strings.
 *
 * @todo come up with a better name.  It's confusing since there is already robot_movement_interface/Command
 *
 * Commands contain a list of pairs of strings.  The pairs represent a command/parameters with optional values.  At
 * command string (with optional values) is required.  Additional parameter pairs can be added.  toString is used to
 * create the actual string to the robot.  << is overridden for easier stream console output.
 *
 * Default string format:
 * <command>[ : <values>]; [\<param>[: <values>];]
 *
 * Examples: \n
 * "ptp : 1 2 3 4 5 6;"  A ptp move \n
 * "ptp : 1 2 3 4 5 6; speed : 100;" A ptp move with the speed parameter specified
 */
class Command
{
public:
  using CommandEntry = std::pair<std::string, std::string>;

  using FullCommand = std::vector<CommandEntry>;

  /// Choose which socket to send over.  Currently only Cmd will do anything.  I'm not sure this will remain.
  enum CommandType
  {
    Get,  //!< Get Commands that should return almost immediately.  get joint position, get digital io, etc
    Cmd   //!< Cmd Commands that might take some time.  Moving, waiting on inputs, etc
  };

  Command(CommandType type = CommandType::Cmd) : type_(type)
  {
    makeCommand(type, "", "");
  }

  Command(CommandType type, const std::string& command, std::string params = "") : type_(type)
  {
    makeCommand(type, command, params);
  }

  Command(CommandType type, const std::string& command, const std::vector<float>& floatVec) : type_(type)
  {
    makeCommand(type, command, Command::paramsToString(floatVec));
  }

  //  Command(const Command &other)
  //  {
  //    this->full_command_ = other.full_command_;
  //    this->type_ = other.type_;
  //  }
  //
  //  Command(Command&& other) :full_command_(std::move(other.full_command_)), type_(other.type_)
  //  {
  //
  //  }

  /**
   * \brief Sets up the command.
   *
   * Sets the first element of full_command_ to (command, params).  It can optionally remove all existing parameters.
   * @param type Cmd or Get
   * @param command command string
   * @param command_vals parameters for the command
   * @param erase_params erase the full_command_ before setting
   */
  void makeCommand(CommandType type, std::string command, std::string command_vals, bool erase_params = false);

  virtual ~Command()
  {
    // std::cout << "Destroying: " << this->toString() << std::endl;
  }

  /**
   * Sets the command and values without modifying the parameters or type
   *
   * @param command command string
   * @param command_vals parameters for the command
   */
  void setCommand(std::string command, std::string command_vals)
  {
    makeCommand(type_, command, command_vals);
  }

  /**
   * Add a parameter and values
   *
   * @param param
   * @param param_vals
   */
  void addParam(std::string param, std::string param_vals);

  /**
   * \brief Prepare the string to send to the robot.
   *
   * @todo rethink this now that option params can be entered.  Or just make it virtual and leave it for someone else to
   * think about!
   * @return The full, formatted string that will be send to the robot
   */
  virtual std::string toString(bool append_newline = true) const;

  /**
   * Check the reponse of a command.  By default it just checks if the response is "error".
   *
   * @param response The string returned by the robot
   * @return True if the response is OK
   */
  virtual bool checkResponse(std::string& response) const;

  /**
   * Converts a float vector into a string of values separated by spaces.  Removes trailing zeroes
   *
   * @param floatVec vector of floats
   * @param precision default 4
   * @return string of values separated by spaces
   */
  static std::string paramsToString(const std::vector<float>& floatVec, int precision = 4);

  /**
   * Used to have an inheritance based << override.
   *
   * @param o operator<<(std::ostream& o, T t)
   * @return ostream containing the results of toString without a newline
   */
  virtual std::ostream& dump(std::ostream& o) const
  {
    std::string str = toString(false);
    o << str;
    return o;
  }

  // Getters/setters

  CommandType getType() const;
  void setType(CommandType type);
  std::string getCommand() const;

  int getCommandId() const;
  void setCommandId(int commandId);

protected:
  FullCommand full_command_;

  /// Used in the /command_result response
  int command_id_;

  CommandType type_;
};

inline std::ostream& operator<<(std::ostream& o, const Command& cmd)
{
  return cmd.dump(o);
}

class CommandHandler;
using CommandPtr = std::shared_ptr<Command>;

/**
 * \brief Handle robot_movement_interface::Command and create Commands that are ready to send to the robot.
 *
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
  typedef std::function<CommandPtr(const robot_movement_interface::Command&)> CommandHandlerFunc;

  CommandHandler() : handler_name_("Base CommandHandler")
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
  // CommandHandler(const robot_movement_interface::Command &cmd_msg);

  /**
   * Construct a new CommandHandler that will call a std::function.  Used to quickly create a new handler without having
   to
   * create a new class.
   *
   * Example:
   *   CommandHandler chtest(cmd, [](const robot_movement_interface::Command& cmd_msg)
       {
         return Command(Command::CommandType::Cmd, cmd_msg.command_type, cmd_msg.pose_type);
       });
   *
   * @param sample_command The sample command to match
   * @param f a std::function/lambda that takes an rmi::Command returns a telnet Command
   */
  CommandHandler(const robot_movement_interface::Command& sample_command, CommandHandlerFunc f);

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
  bool operator==(const robot_movement_interface::Command& cmd_msg);

  /**
   * \deprecated replaced by virtual CommandPtr processMsg.
   *
   * Processes a robot_movement_interface::Command.  Override this method in extended classes.
   * The message should have already been checked for relevance.
   * This base version will call the stored std::function.
   *
   * @param cmd_msg The message received from the command_list topic
   * @param telnet_cmd The command to send to the robot
   * @return True if OK
   */
  //  virtual bool processMsg(const robot_movement_interface::Command &cmd_msg, Command &telnet_cmd) const
  //  {
  //    if (!process_func_)
  //    {
  //      ROS_ERROR_STREAM("Base CommandHandler::processMsg was called but the process function was not set!");
  //      return false;
  //    }
  //
  //    auto ret = process_func_(cmd_msg);
  //    telnet_cmd = *ret;
  //
  //    //telnet_cmd = process_func_(cmd_msg);
  //    return telnet_cmd.getCommand().compare("error") != 0;
  //
  //  }

  /**
   * Create a new std::shared_ptr<Command>.
   * The base version will create a base shared_ptr then call processMsg(cmd_msg, telnet_command)
   *
   * @param cmd_msg The message received from the command_list topic
   * @return a new CommandPtr.  nullptr if processing the message failed.
   */
  virtual CommandPtr processMsg(const robot_movement_interface::Command& cmd_msg) const;

  void setProcFunc(CommandHandlerFunc& f)
  {
    process_func_ = f;
  }

  virtual std::string getName() const
  {
    return handler_name_;
  }

  const robot_movement_interface::Command& getSampleCommand() const;

  virtual std::ostream& dump(std::ostream& o) const;

protected:
  robot_movement_interface::Command sample_command_;

  std::string handler_name_;

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
  using CommandHandlerPtrVec = std::vector<std::unique_ptr<CommandHandler>>;

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
  virtual void initialize(const std::vector<std::string>& joints) = 0;

  /**
   * Create commands and put them in the command_handlers_ vector.
   */
  virtual void registerCommands() = 0;

  /**
   * Get the version string.  This will be checked against the string returned by the robot
   * @return version string
   */
  virtual const std::string& getVersion() = 0;

  /**
   * Get the registered handlers
   * @return a reference to the vector of handlers
   */
  CommandHandlerPtrVec& handlers()
  {
    return command_handlers_;
  }

  /**
   * Search through the registered command handlers to find one that matches this message
   *
   * @param msg_cmd robot_movement_interface::Command that was received
   * @return const CommandHandler* that matched the msg_cmd.  nullptr if no match found
   */
  const CommandHandler* findHandler(const robot_movement_interface::Command& msg_cmd);

protected:
  CommandHandlerPtrVec command_handlers_;
};

using CommandRegisterPtr = std::shared_ptr<CommandRegister>;

}  // namespace rmi_driver

#endif /* INCLUDE_COMMANDS_H_ */
