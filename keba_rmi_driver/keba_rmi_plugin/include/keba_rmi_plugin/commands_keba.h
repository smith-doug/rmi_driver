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

#include "rmi_driver/commands.h"

#include <vector>
#include <functional>



/*
 * Current format for commands:
 * <command name> : <params> [<dyn/eventually ovl> : <params>]
 * Right now there is just a space between the required/optional params.
 * Maybe a ; would be good after each command segment?
 */

namespace keba_rmi_driver
{
class KebaCommands : public CommandRegister
{
public:

  KebaCommands();

  void initialize();

  void initialize(const std::vector<std::string> &joints);

  void registerCommands();

protected:
  int num_main_joints_;
  int num_aux_joints_;

  bool commands_registered_;

};

/*
 * Handler for PTP to joint positions
 * command_type: PTP
 * pose_type: JOINTS
 * pose len: 7
 * velocity_type: % not currently used
 * velocity len: 7 not currently used
 */
class KebaCommandPtpJoints : public CommandHandler
{
public:
  KebaCommandPtpJoints();

  bool processMsg(const robot_movement_interface::Command &cmd_msg, Command &telnet_cmd) const override;

};

/**
 * Linear move with a quaternion orientation.
 *
 * Required:
 *   command_type: LIN
 *   pose_type: QUATERNION
 *   pose len: 7
 * Optional: *
 *   if(velocity_type == dyn)  (a full Keba dynamic)
 *     velocity: [velAxis, accAxis, decAxis, jerkAxis, vel, acc, dec, jerk, velOri, accOri, decOri, jerkOri]
 *
 *   if(velocity_type == %)
 *     @todo
 *
 *
 */
class KebaCommandLinQuat : public CommandHandler
{
public:

  KebaCommandLinQuat();

  bool processMsg(const robot_movement_interface::Command &cmd_msg, Command &telnet_cmd) const override;

};

class KebaCommandLinEuler : public CommandHandler
{
public:

  KebaCommandLinEuler();

  bool processMsg(const robot_movement_interface::Command &cmd_msg, Command &telnet_cmd) const override;

};

} // namespace keba_rmi_driver

#endif /* INCLUDE_COMMANDS_KEBA_H_ */
