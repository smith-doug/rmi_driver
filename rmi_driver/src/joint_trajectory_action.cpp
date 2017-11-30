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
JointTrajectoryAction::JointTrajectoryAction(std::string ns, const std::vector<std::string> &joint_names)
  : action_server_(nh_, ns + "/joint_trajectory_action", boost::bind(&JointTrajectoryAction::goalCB, this, _1),
                   boost::bind(&JointTrajectoryAction::cancelCB, this, _1), false)
  , conf_joint_names_(joint_names)
{
  pub_rmi_ = nh_.advertise<robot_movement_interface::CommandList>(ns + "/command_list", 1);

  sub_rmi_ = nh_.subscribe("command_result", 1, &JointTrajectoryAction::subCB_CommandResult, this);

  action_server_.start();
}

void JointTrajectoryAction::test(JointTractoryActionServer::GoalHandle &gh)
{
  auto &traj = gh.getGoal()->trajectory;

  auto &joint_names = traj.joint_names;
  std::vector<size_t> mapping;

  auto boost_perm_test = boost::algorithm::is_permutation(conf_joint_names_, joint_names.begin());
  ROS_INFO_STREAM("####boost_perm_test: " << boost_perm_test);

  for (auto &&name : conf_joint_names_)
  {
    auto idx = std::find(joint_names.begin(), joint_names.end(), name) - joint_names.begin();
    mapping.push_back(idx);
  }
  if (mapping.size() != joint_names.size())
  {
    ROS_INFO_STREAM("rejected");
    gh.setRejected();
    return;
  }

  for (auto &&idx : mapping)
  {
    ROS_INFO_STREAM(idx);
  }

  robot_movement_interface::CommandList cmd_list;

  cmd_id_ = 0;
  for (auto &&point : traj.points)
  {
    robot_movement_interface::Command cmd;
    cmd.command_id = cmd_id_++;

    cmd.command_type = "PTP";
    cmd.pose_type = "JOINTS";

    // boost::transform(mapping, std::back_inserter(cmd.pose), [&](int i) { return point.positions[i]; });

    //    std::transform(mapping.begin(), mapping.end(), std::back_inserter(cmd.pose),
    //                   [&](int i) { return point.positions[i]; });

    // auto begin = boost::make_permutation_iterator(point.positions.begin(), mapping.begin());
    auto begin_zip = boost::make_zip_iterator(boost::make_tuple(point.positions.begin(), point.velocities.begin()));
    auto end_zip = boost::make_zip_iterator(boost::make_tuple(point.positions.end(), point.velocities.end()));

    auto begin = boost::make_permutation_iterator(point.positions.begin(), mapping.begin());
    auto end = boost::make_permutation_iterator(point.positions.end(), mapping.end());

    // std::copy(begin, end, std::back_inserter(cmd.pose));

    cmd.pose = sortVector(mapping, point.positions);

    if (point.velocities.size() == mapping.size())
    {
      cmd.velocity_type = "ROS";

      cmd.velocity = sortVector(mapping, point.velocities);
      //      std::transform(mapping.begin(), mapping.end(), std::back_inserter(cmd.velocity),
      //                     [&](int i) { return point.velocities[i]; });
    }

    if (point.accelerations.size() == mapping.size())
    {
      cmd.acceleration_type = "ROS";
      std::transform(mapping.begin(), mapping.end(), std::back_inserter(cmd.acceleration),
                     [&](int i) { return point.accelerations[i]; });
    }

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
  robot_movement_interface::CommandList cmd_list;
  robot_movement_interface::Command cmd;
  cmd.command_type = "ABORT";

  cmd_list.commands.push_back(cmd);
  cmd_list.replace_previous_commands = true;

  pub_rmi_.publish(cmd_list);

  ROS_INFO_STREAM("CancelCB");
  if (gh == active_goal_)
    active_goal_.setCanceled();
}

}  // namespace rmi_driver
