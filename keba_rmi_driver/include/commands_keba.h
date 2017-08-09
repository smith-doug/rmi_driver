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

//Register command handlers for Keba controls

#ifndef INCLUDE_COMMANDS_KEBA_H_
#define INCLUDE_COMMANDS_KEBA_H_

#include "commands.h"

#include <vector>
#include <functional>

namespace keba_rmi_driver
{
class KebaCommands : public CommandRegister
{
public:

  KebaCommands();

  void registerCommands();



};

class KebaCommandPtpJoints : public CommandHandler
{
public:
  KebaCommandPtpJoints();

  bool processMsg(const robot_movement_interface::Command &cmd_msg, Command &telnet_cmd) override;

  //Command operator()(const robot_movement_interface::Command &msg_cmd);
};

class KebaCommandLinQuat : public CommandHandler
{
public:

  KebaCommandLinQuat();

  bool processMsg(const robot_movement_interface::Command &cmd_msg, Command &telnet_cmd) override;

  //Command operator()(const robot_movement_interface::Command &msg_cmd);
};

class KebaCommandLinEuler : public CommandHandler
{
public:

  KebaCommandLinEuler();

  bool processMsg(const robot_movement_interface::Command &cmd_msg, Command &telnet_cmd) override;

  //Command operator()(const robot_movement_interface::Command &msg_cmd);

};

} // namespace keba_rmi_driver

#endif /* INCLUDE_COMMANDS_KEBA_H_ */
