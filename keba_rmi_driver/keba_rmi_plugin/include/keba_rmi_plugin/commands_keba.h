/*
 * Copyright (c) 2017, Doug Smith
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES
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

// Register command handlers for Keba controls
#ifndef INCLUDE_COMMANDS_KEBA_H_
#define INCLUDE_COMMANDS_KEBA_H_

#include "rmi_driver/commands.h"

#include <functional>
#include <vector>

/*
 * Current format for commands:
 * <command name>[ : <params>];[<dyn/eventually ovl> [: <params>;]]
 * There is a ; after each command and parameter group.  If there are no parameters, there is no additional :.
 * Examples:
 * ptp joints : 1 2 3 4 5 6;
 * ptp joints : 1 2 3 4 5 6; dyn : 1 2 3 4 5 6 7 8 9;
 * get version;
 *
 */

using namespace rmi_driver;

/**
 * @page KebaTelnetTypes Keba Telnet Types
 * \tableofcontents
 * This section documents the actual string that are send to a Keba controller.\n
 * \ref Poses Types of poses\n
 * \ref Dynamics Types of dynamics
 *
 *
 * \section Poses
 * JOINTS: A list of joint positions in ROS units (radians, meters) \n
 * QUATERNION: A cartesian position with the orientation specified as a quaterniom. The form is "x y z w xx yy zz".  The
 * x/y/z coordinates are given in mm.  Ex: \n
 * EULER_INTRINSIC_ZYX: A cartesian position with the orientation in euler angles.  The form is "x y z zz, yy, zz'" The
 * x/y/z coordinates are in mm and rotations are in degrees.  @todo check if EULER_INTRINSIC_ZYX
 * is actually
 * the type keba uses.  There are so many ways to specify them like ZYZ, XYZ, etc
 *
 * \section Dynamics
 * Dyn: A normal Keba dynamic.  [velAxis(0..100->), accAxis, decAxis, jerkAxis, vel(mm/s->),
 * acc, dec, jerk, velOri(deg/s->), accOri, decOri, jerkOri]
 *
 *
 * @page KebaRmiTypes Keba RMI message types
 * \tableofcontents
 * This section describes the robot_movement_interface::Command structures this plugin responds to.\n
 * \ref KebaRmiTypesPoses
 *
 * \ref KebaRmiTypesDynamics
 *
 * \section KebaRmiTypesPoses Pose types and values
 * \par pose_type: \n
 * QUATERNION:  A cartesian position with position values and orientation both in
 * ROS units (meters, quaternion).  [x, y, z, rw, rx, ry, rz].\n
 * EULER_INTRINSIC_ZYX: A cartesian position with position values in ROS units (meters) and orientation represented by a
 * Euler rotation in degrees. [x, y, z, rz, ry, rz']\n
 * JOINTS: Joints positions in ROS units.
 *
 * \section KebaRmiTypesDynamics Types of speeds/accelerations
 * \par velocity_type:
 * DYN: A full Keba dynamic.  This specifies both the velocity and accelerations.  [velAxis(0..100->), accAxis, decAxis,
 * jerkAxis, vel(mm/s->),
 * acc, dec, jerk, velOri(deg/s->), accOri, decOri, jerkOri]\n
 * ROS: ROS style speeds for each joint.  Cannot be used with Lin moves.  Using this without also specifying ROS as the
 * acceleration_type could lead to undesired behavior.  Used by the standard joint_trajectory_action.
 *
 * \par acceleration_type:
 * ROS : ROS style accelerations for each joint.  Cannot be used with Lin moves.  Using this without also specifying ROS
 * as the
 * velocity_type could lead to undesired behavior.  Used by the standard joint_trajectory_action.
 *
 *
 */

/*
 * This plugin is used to send command to a Keba robot.
 *
 * PoseTypes:
 *
 * JOINTS: A list of joint positions in ROS units (radians, meters) \n
 * QUATERNION: A cartesian position with the orientation specified as a quaterniom. The form is "x y z w xx yy zz".  The
 * x/y/z coordinates are given in mm.\n
 * EULER_INTRINSIC_ZYX: A cartesian position with the orientation in euler angles.  @todo check if these
 * EULER_INTRINSIC_ZYX are actually
 * the type keba uses.  There are so many ways to specify them like ZYZ, XYZ, etc
 *
 * Dynamic Types:
 */

/**
 * \brief This plugin is used to send command to a Keba robot.
 *
 *
 */
namespace keba_rmi_plugin
{
/**
 * \brief A Keba Telnet Command.  Currently no different than the base.
 */
class KebaCommand : public Command
{
public:
  KebaCommand(CommandType type = CommandType::Cmd)
  {
    type_ = type;
  }

  //  std::string toString(bool append_newline = true) const override
  //  {
  //    std::cout << "overloaded KebaCommand::toString()\n";
  //    return Command::toString(append_newline);
  //  }
};

/**
 * \brief class that holds all of the registered commands and some joint information.
 */
class KebaCommandRegister : public CommandRegister
{
public:
  KebaCommandRegister();

  /**
   * \brief Set any local variables.  Make sure you call registerCommands.
   * @param joints vector of joint names
   */
  void initialize(const std::vector<std::string> &joints) override;

  const std::string &getVersion() override
  {
    static std::string version("0.0.3");
    return version;
  }

  /// vector of joint names.  @todo move it to CommandRegister?  Could there ever be a robot where they don't want to
  /// specify joint names?
  std::vector<std::string> joint_names_;

protected:
  void registerCommands() override;

  int num_main_joints_;
  int num_aux_joints_;

  bool commands_registered_;
};

class KebaCommandHandler : public CommandHandler
{
public:
  KebaCommandHandler()
  {
    this->command_register_ = 0;
    sample_command_.pose_type = "NOT_SET";
  }

  const KebaCommandRegister *getCommandRegister() const
  {
    return (KebaCommandRegister *)command_register_;
  }

protected:
};

/**
 * \brief Linear move to Joint or Cartesian positions
 *
 * \par Required:
 *   command_type: LIN\n
 *
 *   pose_type: JOINTS|QUATERNION|EULER_INTRINSIC_ZYX.
 *   For details see \link KebaRmiTypesPoses Accepted poses\endlink
 *
 * \par Optional:
 * velocity_type: DYN|ROS.  For details see \link KebaRmiTypesDynamics Accepted dynamics \endlink\n
 *
 * \par Examples:\n
 * 1. Linear move to a joint position with no speed specified.\n
 *
 * \code
 * command_type: 'LIN'
 * pose_type: 'JOINTS'
 * pose: [1.0, -2.1, -1.3, -1.4, 1.5, 0]
 * \endcode
 * Command::toString(): lin joints : 1.0 -2.1 -1.3 -1.4 1.5 0;
 *
 * 2. Linear move to a quaternion position with a DYN velocity
 * \code
 * command_type: 'LIN'
 * pose_type: 'QUATERNION'
 * pose: [0.3, -0.6, 0.365, 0, 0, 1, 0]
 * velocity_type: 'DYN'
 * velocity: [100, 100, 100, 100, 250, 1000, 1000, 10000, 1000, 10000, 10000, 100000]
 * \endcode
 * Command::toString(): lin quaternion : 300 -600 365 0 0 1 0; dyn : 100 100 100 100 250 1000 1000 10000 1000 10000
 * 10000 100000;
 *
 *
 */
class KebaCommandLin : public KebaCommandHandler
{
public:
  KebaCommandLin();

  CommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

/**
 * \brief PTP moves to Joint or Cartesian positons
 *
 * Required:
 *  command_type: PTP
 *  pose_type   : JOINTS|QUATERNION|EULER_INTRINSIC_ZYX
 *  pose:
 *    joints: [joint positions] with size >= size of joint_names_ in the CommandRegister
 *    quaternion: x y z w x y z
 *
 *
 * Optional:
 *  -Keba dynamic:
 *    velocity_type: DYN
 *    velocity: [velAxis(0..100->), accAxis, decAxis, jerkAxis, vel(mm/s->),
 * acc, dec, jerk, velOri(deg/s->), accOri, decOri, jerkOri]
 *      outputs:  dyn : <values>
 *  -Ros velocities:
 *    velocity_type: ROS
 *    velocity: same as JointTrajectoryPoint velocities
 *      outputs: velros : <values>
 *  -Ros accelerations:
 *    acceleration_type: ROS
 *    acceleration: Same as ointTrajectoryPoint accelerations
 *
 */
class KebaCommandPtp : public KebaCommandHandler
{
public:
  KebaCommandPtp();

  CommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

/**
 * Set a setting for all future unspecified moves
 * Required: \n
 *   command_type: DYN
 *   velocity_type: DYN
 *   velocity: [velAxis(0..100->), accAxis, decAxis, jerkAxis, vel(mm/s->),
 *    acc, dec, jerk, velOri(deg/s->), accOri, decOri, jerkOri]
 *
 */
class KebaCommandDyn : public KebaCommandHandler
{
public:
  KebaCommandDyn();

  CommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

class KebaCommandAbort : public CommandHandler
{
public:
  KebaCommandAbort();

  CommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

class KebaCommandSync : public CommandHandler
{
public:
  KebaCommandSync();

  CommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

class KebaCommandWait : public CommandHandler
{
public:
  KebaCommandWait();

  CommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

}  // namespace keba_rmi_plugin

#endif /* INCLUDE_COMMANDS_KEBA_H_ */
