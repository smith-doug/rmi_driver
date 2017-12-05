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
#include <boost/algorithm/cxx11/is_permutation.hpp>
#include <boost/range/algorithm.hpp>
//#include <boost/range/algorithm/transform.hpp>
#include <boost/iterator/permutation_iterator.hpp>
#include <boost/iterator/zip_iterator.hpp>

#include <vector>

namespace rmi_driver
{
JointTrajectoryAction::JointTrajectoryAction(std::string ns, const std::vector<std::string> &joint_names,
                                             JtaCommandHandler *jta_handler)
  : action_server_(nh_, ns + "/joint_trajectory_action", boost::bind(&JointTrajectoryAction::goalCB, this, _1),
                   boost::bind(&JointTrajectoryAction::cancelCB, this, _1), false)
  , conf_joint_names_(joint_names)
  , ns_(ns)
  , jta_handler_(jta_handler)
{
  pub_rmi_ = nh_.advertise<robot_movement_interface::CommandList>(ns + "/command_list", 1);

  sub_rmi_ = nh_.subscribe("command_result", 1, &JointTrajectoryAction::subCB_CommandResult, this);

  action_server_.start();
}

void JointTrajectoryAction::test(JointTractoryActionServer::GoalHandle &gh)
{
  trajectory_msgs::JointTrajectory traj_sorted;
  auto &traj = gh.getGoal()->trajectory;

  auto &joint_names = traj.joint_names;

  std::vector<size_t> mapping;  // mapping[0] will contain the position of joint0 in the vectors

  for (auto &&name : conf_joint_names_)
  {
    auto idx = std::find(joint_names.begin(), joint_names.end(), name) - joint_names.begin();
    mapping.push_back(idx);
  }
  if (mapping.size() != joint_names.size())
  {
    ROS_ERROR_STREAM("rejected");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "mapping.size() != joint_names.size()");
    return;
  }

  traj_sorted.header = traj.header;
  traj_sorted.joint_names = sortVectorByIndices<std::string>(mapping, traj.joint_names);

  robot_movement_interface::CommandList cmd_list;

  cmd_id_ = 0;

  try
  {
    for (auto &&point : traj.points)
    {
      trajectory_msgs::JointTrajectoryPoint jtp;

      jtp.positions = sortVectorByIndices<double>(mapping, point.positions);
      jtp.velocities = sortVectorByIndices<double>(mapping, point.velocities);
      jtp.accelerations = sortVectorByIndices<double>(mapping, point.accelerations);
      jtp.effort = sortVectorByIndices<double>(mapping, point.effort);
      jtp.time_from_start = point.time_from_start;

      traj_sorted.points.push_back(jtp);

      //      robot_movement_interface::Command cmd;
      //      cmd.command_id = cmd_id_++;
      //
      //      cmd.command_type = "PTP";
      //      cmd.pose_type = "JOINTS";
      //
      //      cmd.pose = sortVectorByIndices<float>(mapping, point.positions);
      //
      //      if (point.velocities.size() == mapping.size())
      //      {
      //        cmd.velocity_type = "ROS";
      //        cmd.velocity = sortVectorByIndices<float>(mapping, point.velocities);
      //      }
      //
      //      if (point.accelerations.size() == mapping.size())
      //      {
      //        cmd.acceleration_type = "ROS";
      //        cmd.acceleration = sortVectorByIndices<float>(mapping, point.accelerations);
      //      }
      //      cmd_list.commands.push_back(std::move(cmd));
    }
  }
  catch (const std::runtime_error &error)
  {
    gh.setRejected();
    abort(error.what());
  }

  cmd_list = jta_handler_->processJta(traj_sorted);

  robot_movement_interface::Command cmd;
  // cmd.command_id = cmd_id_;
  cmd.command_id = cmd_list.commands.back().command_id + 1;
  cmd_id_ = cmd.command_id;
  cmd.command_type = "WAIT";
  cmd.pose_type = "IS_FINISHED";
  cmd_list.commands.push_back(cmd);

  // std::transform(mapping.begin(), mapping.end(), sorted.begin(), [&](int i) { return pt.positions[i]; });

  // std::transform(mapping.begin(), mapping.end(), std::back_inserter(cmd.pose), [&](int i) { return pt.positions[i];
  // });

  // ROS_INFO_STREAM(cmd_list);
  gh.setAccepted();

  pub_rmi_.publish(cmd_list);
  JointTractoryActionServer::Result res;
}

void JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle gh)
{
  active_goal_ = gh;

  test(gh);
}

void JointTrajectoryAction::subCB_CommandResult(const robot_movement_interface::ResultConstPtr &msg)
{
  if (msg->command_id == cmd_id_ && active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
    active_goal_.setSucceeded();
}

void JointTrajectoryAction::cancelCB(JointTractoryActionServer::GoalHandle gh)
{
  ROS_INFO_STREAM("CancelCB");
  if (gh == active_goal_)
  {
    abort("JointTrajectoryAction::cancelCB called");
  }
}

void JointTrajectoryAction::abort(const std::string &error_msg)
{
  robot_movement_interface::CommandList cmd_list;
  robot_movement_interface::Command cmd;
  cmd.command_type = "ABORT";

  cmd_list.commands.push_back(cmd);
  cmd_list.replace_previous_commands = true;

  pub_rmi_.publish(cmd_list);

  auto status = active_goal_.getGoalStatus().status;
  if (status == actionlib_msgs::GoalStatus::PENDING || status == actionlib_msgs::GoalStatus::RECALLING ||
      status == actionlib_msgs::GoalStatus::ACTIVE || status == actionlib_msgs::GoalStatus::PREEMPTING)
    active_goal_.setCanceled();

  ROS_INFO_STREAM(ns_ << " JointTrajectoryAction::abort: " << error_msg);
}

}  // namespace rmi_driver
