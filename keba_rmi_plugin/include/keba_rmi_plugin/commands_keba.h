/*
 * Copyright (c) 2017, Doug Smith, KEBA Corp
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
 *
 * @page KebaTelnetTypes Keba RobotCommand Telnet Types
 * \tableofcontents
 * This section documents the actual strings that are send to a Keba controller.\n
 * \ref Moves Types of moves\n
 * \ref Poses Types of poses\n
 * \ref Dynamics Types of dynamics\n
 * \ref ExamplesKebaTelnetTypes\n
 *
 *
 * \section Moves
 * lin: Linear move.  Calls the Lin() macro.\n
 * ptp: Point to point move.  Calls the PTP() macro.\n
 *
 * \section Poses
 * joints: A list of joint positions in ROS units (radians, meters).  See trajectory_msgs/JointTrajectoryPoint \n
 * quaternion: A cartesian position with the orientation specified as a quaterniom. The form is "x y z w xx yy zz".  The
 * x/y/z coordinates are given in mm.  \n
 * euler_intrinsic_zyx: A cartesian position with the orientation in euler angles.  The form is "x y z zz, yy, zz'" The
 * x/y/z coordinates are in mm and rotations are in degrees.  @todo check if EULER_INTRINSIC_ZYX
 * is actually
 * the type keba uses.  There are so many ways to specify them like ZYZ, XYZ, etc
 *
 * \section Dynamics
 * dyn: A normal Keba dynamic.  [velAxis(0..100->), accAxis, decAxis, jerkAxis, vel(mm/s->),
 * acc, dec, jerk, velOri(deg/s->), accOri, decOri, jerkOri]\n
 * velros/accros: ROS velocity/accelerations.  Only usable with PTP motion.  See trajectory_msgs/JointTrajectoryPoint.\n
 *
 * \section ExamplesKebaTelnetTypes Examples
 * A PTP move to a joint position with ROS accel and velocity.
 * \code
 * ptp joints : 1 2 3 4 5 6;velros : 0.1 0.2 0.3 0.4 0.5 0.6;accros : 1.1 1.2 1.3 1.4 1.5 1.6;
 * \endcode
 *
 * A Linear move to a joint position with Keba dynamic
 * \code
 * lin joints : 1 -2.1 -1.3 -1.4 1.5 0 -0.3;dyn : 100 100 100 100 500 1000 1000 10000 1000 10000 10000 100000;
 * \endcode
 *
 * A PTP move to a Quaternion with Keba dynamic
 * \code
 * ptp quaternion : 500 -600 365 0 0 1 0;dyn : 10 100 100 100 50 1000 1000 10000 1000 10000 10000 100000;
 * \endcode
 *
 *
 *
 * @page KebaRmiTypes Keba robot_movement_interface::Command formats
 * \tableofcontents
 * This section describes the shared robot_movement_interface::Command structures this plugin responds to.\n
 * \ref KebaRmiTypesPoses
 *
 * \ref KebaRmiTypesDynamics
 *
 * \section KebaRmiTypesPoses Pose types and values
 * \par pose_type:
 * QUATERNION:  A cartesian position with position values and orientation both in
 * ROS units (meters, quaternion).  [x, y, z, rw, rx, ry, rz].\n
 * EULER_INTRINSIC_ZYX: A cartesian position with position values in ROS units (meters) and orientation
 represented by a
 * Euler rotation in degrees. [x, y, z, rz, ry, rz']\n
 * JOINTS: Joints positions in ROS units.\n
 * @todo it's ZYZ, check it
 *
 * \section KebaRmiTypesDynamics Types of speeds/accelerations
 * \par velocity_type:
 * DYN: A full Keba dynamic.  This specifies both the velocity and accelerations.  [velAxis(0..100->), accAxis, decAxis,
 * jerkAxis, vel(mm/s->),
 * acc, dec, jerk, velOri(deg/s->), accOri, decOri, jerkOri]\n
 * Example:
 * \code
 * velocity_type: DYN
   velocity: [100.0, 100.0, 100.0, 100.0, 50.0, 1000.0, 1000.0, 10000.0, 1000.0, 10000.0, 10000.0, 100000.0]
 * \endcode
 * ROS: ROS style speeds for each joint.  Cannot be used with Lin moves.  Using this without also specifying ROS as the
 * acceleration_type could lead to undesired behavior.  Used by the standard joint_trajectory_action.
 *
 * \par acceleration_type:
 * ROS : ROS style accelerations for each joint.  Cannot be used with Lin moves.  Using this without also specifying ROS
 * as the
 * velocity_type could lead to undesired behavior.  Used by the standard joint_trajectory_action.
 *
 * \par blending_type:
 * % or OVLREL: Relative overlapping.  A number between 0 and 200 [0-200]\n
 * OVLSUPPOS: Superposition overlapping.  A number between 0 and 200 [0-200]\n
 * Example:
 * \code
 *  blending_type: 'OVLREL'
    blending: [50]
 * \endcode
 * OVLABS: Absolute overlapping.  5 numbers.  [posDist, oriDist, linAxDist, rotAxDist, vConst(boolean)\n
 * \code
 * blending_type: 'OVLABS'
   blending: [10, 360, 1000, 360, 0]
 * \endcode
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
 * \brief This plugin is used to send command to a Keba robot.\n
 * Available commands:\n
 * LIN KebaCommandLin\n
 * PTP KebaCommandPtp\n
 * SETTING KebaCommandSetting\n
 * WAIT KebaCommandWait\n
 * SYNC KebaCommandSync\n
 * FRAME KebaCommandSetFrame\n
 */
namespace keba_rmi_plugin
{
class KebaException : public std::runtime_error
{
public:
  explicit KebaException(const std::string &str) : std::runtime_error(str)
  {
  }
  explicit KebaException(const char *str) : std::runtime_error(str)
  {
  }
};

/**
 * \brief A Keba Telnet Command.  Currently no different than the base.
 */
class KebaCommand : public RobotCommand
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
 * \brief Handles any custom things needed for JointTrajectoryAction
 *
 * In this case, all I'm doing it adding a WaitIsFinished() to the end of the trajectory.  This makes it much simpler to
 * tell when the trajectory is actually done.
 */
class KebaJtaCommandHandler : public JtaCommandHandler
{
public:
  KebaJtaCommandHandler()
  {
    handler_name_ = "Keba JTA";
  }

  /**
   * \brief Add a WaitIsFinished to the end of the trajectory.
   *
   * @param point Last point
   * @param cmd_list Full command list
   */
  void processLastJtaPoint(const trajectory_msgs::JointTrajectoryPoint &point,
                           robot_movement_interface::CommandList &cmd_list) override;
};

/**
 * \brief Class that holds all of the registered commands and some joint information.
 *
 *
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
    static std::string version("0.0.9");
    return version;
  }

  /// vector of joint names.  @todo move it to CommandRegister?  Could there ever be a robot where they don't want to
  /// specify joint names?
  std::vector<std::string> joint_names_;

protected:
  /// \brief Create and load all the CommandHandlers
  void registerCommandHandlers() override;

  /// Number of main joints.  Unused
  int num_main_joints_;
  /// Number of aux joints.  Unused
  int num_aux_joints_;

  /// Prevent commands from being registered again
  bool commands_registered_ = false;
};

/**
 * \brief Extended CommandHandler.  Base class of all command handlers in this plugin.
 *
 * Not doing much beyond what the base already does, but it does allow me to get a
 * pointer to the KebaCommandRegister
 */
class KebaCommandHandler : public CommandHandler
{
public:
  KebaCommandHandler()
  {
    this->command_register_ = 0;
    sample_command_.pose_type = "NOT_SET";
  }

  const KebaCommandRegister *getCommandRegister() const override
  {
    return (KebaCommandRegister *)command_register_;
  }

protected:
};

/**
 * \brief Special command handler used by Connector::getThread()
 *
 * \details This handler implements the standard gets.  @todo define these!\n
 * \par Required:
 * command_type: GET\n
 * pose_type: JOINT_POSITION, TOOL_FRAME, VERSION\n
 *
 *
 * Command::toString will produce one of the following:\n
 * "get joint position;"\n
 *
 * "get version;"
 */
class KebaCommandGet : public KebaCommandHandler
{
public:
  KebaCommandGet();

  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

/**
 * \brief Handles getting and converting the tcp frame
 *
 * This handler will get the current reported robot tcp position and orientation.  It will be converted from the Keba
 * standard ZYZ to the ZYX format robot_movement_interface specifies.
 *
 * \par Required:
 * command_type: GET\n
 * pose_type: TOOL_FRAME\n
 *
 * Command::toString will produce:\n
 * "get tool frame ros;"
 *
 * The controller will respond with "x_in_mm y_in_mm z_in_mm zrot_in_rads yrot_in_rads z'rot_in_rads".  It can convert
 * between degrees/rads easily enough, but reordering the orientation with matrices is too difficult to do on a plc.
 */
class KebaCommandGetToolFrame : public KebaCommandHandler
{
  /// Converts the Keba rotation ZYZ' into ZYX
  class KebaCommandToolFrame : public KebaCommand
  {
  public:
    KebaCommandToolFrame(CommandType type = CommandType::Get);

    /**
     * \brief Change the Keba ZYZ' rotation into ZYX
     * @param response Modified string
     */
    void processResponse(std::string &response) const override;
  };

public:
  KebaCommandGetToolFrame();

  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

class KebaCommandGetStatus : public KebaCommandHandler
{
  /// Converts the Keba rotation ZYZ' into ZYX
  class KebaCommandStatus : public RobotCommandStatus
  {
  public:
    KebaCommandStatus(CommandType type = CommandType::Get);

    /**
     * \brief Change the Keba ZYZ' rotation into ZYX
     * @param response Modified string
     */
    void processResponse(std::string &response) const override;

    void updateData(std::string &response) override;
  };

public:
  KebaCommandGetStatus();

  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

/**
 * \brief Linear move to Joint or Cartesian positions
 *
 * \par Required:
 *   command_type: LIN\n
 *
 *   pose_type: JOINTS|QUATERNION|EULER_INTRINSIC_ZYX.\n
 *   pose: For details see \link KebaRmiTypesPoses Accepted poses\endlink
 *
 * \par Optional:
 * velocity_type: DYN|ROS.  For details see \link KebaRmiTypesDynamics Accepted dynamics \endlink\n
 *
 * \par Examples:
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

  void initialize() override;

  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

/**
 * \brief PTP moves to Joint or Cartesian positons
 *
 * \par Required:
 *  command_type: PTP\n
 *  pose_type   : JOINTS|QUATERNION|EULER_INTRINSIC_ZYX\n
 *  pose: For details see \link KebaRmiTypesPoses Accepted poses\endlink\n
 *
 *  \par Optional:
 *  velocity_type: DYN|ROS\n
 *  velocity: For details see \link KebaRmiTypesDynamics Accepted dynamics \endlink \n\n
 *
 *  acceleration_type: ROS  (Acceleration must be empty if using DYN.  The entire Dynamic is specified in velocty.)\n
 *  acceleration: Empty if using DYN|For details see \link KebaRmiTypesDynamics Accepted dynamics \endlink \n
 *
 * \par Examples:
 * 1. PTP move to a joint position with no speed specified.\n
 *
 * \code
 * command_type: 'PTP'
 * pose_type: 'JOINTS'
 * pose: [1.0, -2.1, -1.3, -1.4, 1.5, 0]
 * \endcode
 * Command::toString(): ptp joints : 1.0 -2.1 -1.3 -1.4 1.5 0;
 *
 * 2. PTP move to a quaternion position with a DYN velocity
 * \code
 * command_type: 'PTP'
 * pose_type: 'QUATERNION'
 * pose: [0.3, -0.6, 0.365, 0, 0, 1, 0]
 * velocity_type: 'DYN'
 * velocity: [100, 100, 100, 100, 250, 1000, 1000, 10000, 1000, 10000, 10000, 100000]
 * \endcode
 * Command::toString(): ptp quaternion : 300 -600 365 0 0 1 0; dyn : 100 100 100 100 250 1000 1000 10000 1000 10000
 * 10000 100000;
 */
class KebaCommandPtp : public KebaCommandHandler
{
public:
  KebaCommandPtp();

  void initialize() override;

  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

/**
 * \brief Calls functions in Kairo Settings category.  Currently Dyn, Ovl
 *
 * This handler is used to call functions found in the Settings category.  You can specify settings in individual
 * messages or combine them.  You must specify at least 1.
 *
 * See \link KebaRmiTypesDynamics Keba Dynamic Types\endlink for more details on the types supported by Dyn and Ovl.
 *
 * \par Required:
 * command_type: SETTING
 *
 *
 * \par Dynamics:
 * Set a Dyn for all future unspecified moves.  This is the same as calling Dyn() on the pendant.  ROS velocities aren't
 * supported.   \n
 * \par Required if setting a dynamic:\n
 * velocity_type: DYN\n
 * velocity: See \link KebaRmiTypesDynamics Keba Dynamic Types.\endlink
 *
 * \par Overlap:
 * Set an Ovl for all future unspecified moves.  This is the same as calling Ovl() on the pendant.  %/OVLREL, OVLSUPPOS,
 * and OVLABS are supported.\n
 * Required if setting an overlap:\n
 * blending_type: %|OVLREL|OVLSUPPOS|OVLABS\n
 * blending: See \link KebaRmiTypesDynamics Keba Dynamic Types\endlink
 *
 *
 * \par Examples:
 * 1. Set a Dynamic\n
 * \code
 * command_type: 'SETTING'
 * velocity_type: DYN
 * velocity: [100, 100, 100, 100, 150, 500, 500, 10000, 360, 900, 900, 3600]
 * \endcode
 * Command::toString(): "setting; dyn : 100 100 100 100 150 500 500 500 10000 360 900 900 3600;"
 *
 * 2. Set an overlap\n
 * \code
 * command_type: 'SETTING'
 * blending_type: 'OVLABS'
 * blending: [10, 360, 1000, 360, 0]
 * \endcode
 * Command::toString(): "setting;ovlabs : 10 360 1000 360 0;"
 *
 * 3. Set both a Dynamic and Overlap\n
 * \code
 * command_type: 'SETTING'
 * velocity_type: DYN
 * velocity: [100, 100, 100, 100, 150, 500, 500, 10000, 360, 900, 900, 3600]
 * blending_type: 'OVLABS'
 * blending: [10, 360, 1000, 360, 0]
 * \endcode
 * Command::toString(): "setting;ovlabs : 10 360 1000 360 0;dyn : 100 100 100 100 150 10000 10000 1000000 10000 10000
 * 10000 100000;"
 */
class KebaCommandSetting : public KebaCommandHandler
{
public:
  KebaCommandSetting();

  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

/**
 * \brief Stops the robot and clears any remaining path.
 *
 * This handler will trigger the AbortMonitor function.  It is sent over the Get channel to immediately stop the robot.
 *
 *
 * \warning Abort should always be called by itself with replace_previous_commands = True.  Not setting
 * replace_previous_commands
 * could lead to undesired behavior.
 *
 * \par Required:
 * command_type: ABORT
 *
 * Command::toString(): "abort;"
 *
 */
class KebaCommandAbort : public KebaCommandHandler
{
public:
  KebaCommandAbort();

  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

class KebaCommandSync : public KebaCommandHandler
{
public:
  KebaCommandSync();

  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

/**
 * \brief Call Kairo Wait functions
 *
 * This handler is used to call functions that begin with "Wait", like WaitIsFinished.  It currently only supports
 * WaitIsFinished.
 *
 * \par REQUIRED:
 * command_type: WAIT\n
 * pose_type: IS_FINISHED
 *
 * \par Examples
 * 1. Call WaitIsFinished()\n
 * \code
 * command_type: WAIT
 * pose_type: IS_FINISHED
 * \endcode
 * Command::toString(): "wait is_finished;"
 */
class KebaCommandWait : public KebaCommandHandler
{
public:
  KebaCommandWait();

  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

/**
 * Call Kairo functions that involve setting frames
 *
 * This handler is used to call functions that operate on frames, like Tool or RefSys.  Currently only Tool is
 * supported.
 *
 * \par REQUIRED:
 * command_type: FRAME\n
 * pose_reference: TOOL\n
 * pose: [x, y, z, a, b, c] in meters/radians.  The orientation is Keba ZYZ.  The units will be converted to mm/degrees
 * before sending.  @todo make the input in keba units too?  It's directly calling Tool with the values passed.
 *
 * \par Examples
 * 1. Call Tool() with a straight 50mm tool.
 * \code
 * command_type: FRAME
 * pose_reference: TOOL
 * pose: [0, 0, .050, 0, 0, 0]
 * \endcode
 * Command::toString(): "frame;tool : 0 0 50 0 0 0;"
 *
 */
class KebaCommandSetFrame : public KebaCommandHandler
{
public:
  KebaCommandSetFrame();

  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override;
};

}  // namespace keba_rmi_plugin

#endif /* INCLUDE_COMMANDS_KEBA_H_ */
