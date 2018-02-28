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
 *  Created on: Sep 12, 2017
 *      Author: Doug Smith
 */

#ifndef INCLUDE_KEBA_RMI_PLUGIN_KEBA_UTIL_H_
#define INCLUDE_KEBA_RMI_PLUGIN_KEBA_UTIL_H_

#include "keba_rmi_plugin/commands_keba.h"

namespace keba_rmi_plugin
{
/**
 * \brief Checks for velocity type DYN and adds the param entry if found
 *
 * @param cmd_msg [in] the whole message
 * @param telnet_cmd [in,out] the telnet command to add the dyn entry to
 * @return true if a keba DYN entry was found
 */
bool processKebaDyn(const robot_movement_interface::Command &cmd_msg, RobotCommand &telnet_cmd);

/**
 * \brief Check for overlaps and add them to params if found.
 *
 * Supported types: %/OVLREL, OVLSUPPOS, OVLABS
 *
 * @param cmd_msg [in] the whole message
 * @param telnet_cmd [in,out] the telnet command to add the dyn entry to
 * @return true if an overlap entry was found
 */
bool processKebaOvl(const robot_movement_interface::Command &cmd_msg, RobotCommand &telnet_cmd);

bool processKebaFrame(const robot_movement_interface::Command &cmd_msg, RobotCommand &telnet_cmd);

/**
 * Check for velo/accel types to be ROS  @todo should I require both?
 * @param cmd_msg
 * @param telnet_cmd
 * @return
 */
bool processRosDyn(const robot_movement_interface::Command &cmd_msg, RobotCommand &telnet_cmd);

double radToDeg(double rad);

/**
 * \brief Process any Aux joint values
 *
 * @todo Think about this.  I don't like this mixing of ros/keba values for a position but can't think of a better way
 * to handle it right now.
 *
 * \details Look for entries matching the format "aux#:###" like "aux1:1234".  The value is directly set as the aux
 * value without any conversion.  The units for the value must match the units on the PLC (degrees, mm).
 * @param cmd_msg
 * @param telnet_cmd
 * @return True if there was some aux value
 */
bool processKebaAux(const robot_movement_interface::Command &cmd_msg, RobotCommand &telnet_cmd);

std::string convertToolFrameStr(const std::string &response);

// bool processKebaPose(const robot_movement_interface::Command &cmd_msg, std::string &pose_value_str);

}  // namespace keba_rmi_plugin

#endif /* INCLUDE_KEBA_RMI_PLUGIN_KEBA_UTIL_H_ */
