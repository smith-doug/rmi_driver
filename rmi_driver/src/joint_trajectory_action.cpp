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
 *  Created on: Nov 29, 2017
 *      Author: Doug Smith
 */

#include <rmi_driver/joint_trajectory_action.h>
#include <vector>

namespace rmi_driver
{
}  // namespace rmi_driver

rmi_driver::JointTrajectoryAction::JointTrajectoryAction(std::string ns, const std::vector<std::string> &joint_names)
  : action_server_(nh_, ns + "/joint_trajectory_action", boost::bind(&JointTrajectoryAction::goalCB, this, _1), false)
  , conf_joint_names_(joint_names)
{
  pub_rmi_ = nh_.advertise<robot_movement_interface::CommandList>(ns + "/command_list", 1);

  sub_rmi_ = nh_.subscribe("command_result", 1, &JointTrajectoryAction::subCB_CommandResult, this);

  action_server_.start();
}

void rmi_driver::JointTrajectoryAction::test(JointTractoryActionServer::GoalHandle &gh)
{
  //  std::vector<std::string> conf_names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  //  "wrist_1_joint",
  //                                          "wrist_2_joint",      "wrist_3_joint",       "rail_to_base" };

  auto &traj = gh.getGoal()->trajectory;

  auto &joint_names = traj.joint_names;
  std::vector<int> mapping;

  for (auto &&name : conf_joint_names_)
  {
    auto idx = std::find(joint_names.begin(), joint_names.end(), name) - joint_names.begin();
    mapping.push_back(idx);
  }

  for (auto &&idx : mapping)
  {
    ROS_INFO_STREAM(idx);
  }

  // trajectory_msgs::JointTrajectoryPoint pt;
  // pt.positions = { 2, 6, 1, 0, 3, 4, 5 };
  // goal.goal.trajectory.points.push_back(pt);

  robot_movement_interface::CommandList cmd_list;

  cmd_id_ = 0;
  for (auto &&point : traj.points)
  {
    robot_movement_interface::Command cmd;
    cmd.command_id = cmd_id_++;

    cmd.command_type = "PTP";
    cmd.pose_type = "JOINTS";
    std::transform(mapping.begin(), mapping.end(), std::back_inserter(cmd.pose),
                   [&](int i) { return point.positions[i]; });

    cmd_list.commands.push_back(std::move(cmd));
  }

  robot_movement_interface::Command cmd;
  cmd.command_id = cmd_id_;
  cmd.command_type = "WAIT";
  cmd.pose_type = "IS_FINISHED";
  cmd_list.commands.push_back(cmd);

  // std::transform(mapping.begin(), mapping.end(), sorted.begin(), [&](int i) { return pt.positions[i]; });

  // std::transform(mapping.begin(), mapping.end(), std::back_inserter(cmd.pose), [&](int i) { return pt.positions[i];
  // });

  // ROS_INFO_STREAM(cmd_list);

  pub_rmi_.publish(cmd_list);
  JointTractoryActionServer::Result res;
}

void rmi_driver::JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle gh)
{
  active_goal_ = gh;
  gh.setAccepted();
  test(gh);
}

void rmi_driver::JointTrajectoryAction::subCB_CommandResult(const robot_movement_interface::ResultConstPtr &msg)
{
  if (msg->command_id == cmd_id_)
    active_goal_.setSucceeded();
}
