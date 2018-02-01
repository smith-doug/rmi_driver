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

#include "keba_rmi_plugin/keba_util.h"
#include <boost/algorithm/string.hpp>
#include <vector>

namespace keba_rmi_plugin
{
// Checks for velocity type DYN and adds the param entry if found
bool processKebaDyn(const robot_movement_interface::Command &cmd_msg, RobotCommand &telnet_cmd)
{
  if (boost::iequals(cmd_msg.velocity_type, "DYN"))
  {
    if (cmd_msg.velocity.size() != 12)
    {
      std::ostringstream oss;
      oss << "keba_rmi_driver::processKebaDyn failed, dyn expects 12 entries but got " << cmd_msg.velocity.size();
      throw KebaException(oss.str());
    }

    telnet_cmd.addParam("dyn", RobotCommand::paramsToString(cmd_msg.velocity));
    return true;
  }
  else
    return false;
}

// Check for overlaps and add them to params if found
bool processKebaOvl(const robot_movement_interface::Command &cmd_msg, RobotCommand &telnet_cmd)
{
  if (cmd_msg.blending_type.length() < 1)
    return false;

  auto blending_type = boost::to_lower_copy(cmd_msg.blending_type);

  auto blending = cmd_msg.blending;

  //% == OVLREL for convenience.
  if (boost::iequals(blending_type, "%"))
    blending_type = "OVLREL";

  // These 2 types both have 1 parameters, a % from 0-200.
  if (boost::iequals(cmd_msg.blending_type, "OVLREL") || boost::iequals(cmd_msg.blending_type, "OVLSUPPOS"))
  {
    if (blending.size() != 1)
      return false;  // Maybe I should throw

    blending[0] = std::round(blending[0]);
    int ovl = std::lround(blending[0]);
    if (ovl < 0 || ovl > 200)
      return false;
  }
  else if (boost::iequals(cmd_msg.blending_type, "OVLABS"))
  {
    // Check stuff.
  }
  else
  {
    return false;
  }

  telnet_cmd.addParam(boost::to_lower_copy(blending_type), RobotCommand::paramsToString(blending));

  return true;
}

bool processKebaFrame(const robot_movement_interface::Command &cmd_msg, RobotCommand &telnet_cmd)
{
  if (cmd_msg.pose_reference.length() < 1)
    return false;

  auto pose_reference = boost::to_lower_copy(cmd_msg.pose_reference);

  auto pose_copy = cmd_msg.pose;

  if (boost::iequals(pose_reference, "TOOL") && pose_copy.size() == 6)
  {
    pose_copy[0] *= 1000.0;
    pose_copy[1] *= 1000.0;
    pose_copy[2] *= 1000.0;
    pose_copy[3] = radToDeg(pose_copy[3]);
    pose_copy[4] = radToDeg(pose_copy[4]);
    pose_copy[5] = radToDeg(pose_copy[5]);

    telnet_cmd.addParam("tool", RobotCommand::paramsToString(pose_copy, 2));
  }
  else
  {
    return false;
  }

  return true;
}

bool processRosDyn(const robot_movement_interface::Command &cmd_msg, RobotCommand &telnet_cmd)
{
  bool ret = false;
  if (cmd_msg.velocity_type.compare("ROS") == 0)
  {
    telnet_cmd.addParam("velros", RobotCommand::paramsToString(cmd_msg.velocity));
    ret = true;
  }
  if (cmd_msg.acceleration_type.compare("ROS") == 0)
  {
    telnet_cmd.addParam("accros", RobotCommand::paramsToString(cmd_msg.acceleration));
    ret = true;
  }
  return ret;
}

double radToDeg(double rad)
{
  return rad * (180.0 / M_PI);
}

// Process any Aux joint values
bool processKebaAux(const robot_movement_interface::Command &cmd_msg, RobotCommand &telnet_cmd)
{
  bool ret = false;
  std::vector<float> aux_values;

  auto param_strings = cmd_msg.additional_parameters;

  for (auto &&str_param : param_strings)
  {
    // Look for aux
    if (boost::istarts_with(str_param, "aux"))
    {
      ret = true;
      std::vector<std::string> str_split;

      // Split it at the :
      boost::split(str_split, str_param, boost::is_any_of(":"), boost::token_compress_on);

      if (str_split.size() != 2)  // There should be 2 strings from the split
      {
        throw KebaException("keba_rmi_plugin::processKebaAux() failed: Size of split was wrong in: " + str_param);
      }

      for (auto &&str : str_split)  // Clean it up
      {
        boost::trim(str);
        boost::to_lower(str);
      }

      try  // Make sure the value is actually a number
      {
        boost::lexical_cast<double>(str_split[1]);
      }
      catch (const boost::bad_lexical_cast &ex)
      {
        throw KebaException("keba_rmi_plugin::processKebaAux() failed: lexical_cast failed:  " +
                            std::string(ex.what()));
      }

      telnet_cmd.addParam(str_split[0], str_split[1]);
    }
  }

  return ret;
}

}  // namespace keba_rmi_plugin
