/*
 * Copyright (c) 2018, Doug Smith, KEBA Corp
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
 *  Created on: Jan 15, 2018
 *      Author: Doug Smith
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <iostream>
#include <memory>

#include <rmi_driver/commands.h>
#include <rmi_driver/connector.h>
#include <rmi_driver/driver.h>
#include <rmi_driver/rotation_utils.h>

using namespace rmi_driver;

namespace rmi_driver
{
class TestDriver : public Driver
{
public:
  // Driver driver_;
  TestDriver() : Driver()
  {
  }

  //  void start()
  //  {
  //    this->start();
  //    // driver_.start();
  //  }
  std::unordered_map<int32_t, std::shared_ptr<Connector>>& getConnMap()
  {
    return this->conn_map_;
  }
};
}

class TestData
{
public:
  DriverConfig config_;
  bool config_loaded_ = false;

  // The CommandRegister that was loaded by the plugin
  CommandRegisterPtr cmd_register_;
  // "The ClassLoader must not go out scope while you are using the plugin."  Keep it alive.
  CmdRegLoaderPtr cmd_reg_loader;

  // When a Driver or a Connector get destroyed before the end of a program, they make things very angry.  Keep them in
  // here to keep them alive between tests.
  std::shared_ptr<TestDriver> pDriver_;
  std::shared_ptr<Connector> connector_;
};

class TestCommandRegister : public CommandRegister
{
public:
  TestCommandRegister()
  {
  }

  void initialize(const std::vector<std::string>& joints) override
  {
    registerCommandHandlers();
  }

  const std::string& getVersion() override
  {
    static std::string version("0.0.7");
    return version;
  }

  bool commands_registered_ = false;

protected:
  /// \brief Create and load all the CommandHandlers
  void registerCommandHandlers() override
  {
  }
};

class CommandGet : public CommandHandler
{
public:
  CommandGet()
  {
    handler_name_ = "KebaCommandGet";

    robot_movement_interface::Command cmd;
    cmd = robot_movement_interface::Command();
    cmd.command_type = "GET";
    cmd.pose_type = "JOINT_POSITION|TOOL_FRAME|VERSION";

    sample_command_ = cmd;
  }

  RobotCommandPtr processMsg(const robot_movement_interface::Command& cmd_msg) const override
  {
    std::string cmd_str = "get ";
    RobotCommandPtr cmd_ptr = std::make_shared<RobotCommand>(RobotCommand::RobotCommand::CommandType::Get);
    if (boost::iequals("JOINT_POSITION", cmd_msg.pose_type))
      cmd_str += "joint position";
    else if (boost::iequals("TOOL_FRAME", cmd_msg.pose_type))
      cmd_str += "tool frame ros";
    else if (boost::iequals("VERSION", cmd_msg.pose_type))
      cmd_str += "version";
    else
      return nullptr;

    cmd_ptr->setCommand(cmd_str, "");
    return cmd_ptr;
  }
};

TestData test_data_;
boost::asio::io_service io_service_;
boost::asio::io_service::work work_(io_service_);
std::thread io_service_thread_;

// TEST(TestSuite, DISABLED_driver_connect)
void driver_connect()
{
  std::vector<std::string> joint_names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                           "wrist_2_joint",      "wrist_3_joint",       "rail_to_base" };

  //  auto shared = std::make_shared<Connector>("/", io_service_, "127.0.0.1", 30000, joint_names, commands, cmh_loader,
  //                                            config_.clear_commands_on_error_);

  ros::NodeHandle nh;
  try
  {
    test_data_.pDriver_->start();

    auto& conn_map = test_data_.pDriver_->getConnMap();
    auto conn_ptr = conn_map.begin()->second;

    robot_movement_interface::Command cmd;
    cmd.command_type = "WAIT";
    cmd.pose_type = "IS_FINISHED";
    auto cmh = conn_ptr->getCommandRegister()->findHandler(cmd);
    EXPECT_TRUE(cmh);

    cmd.command_type = "INVALID";
    cmh = conn_ptr->getCommandRegister()->findHandler(cmd);
    EXPECT_FALSE(cmh);

    EXPECT_TRUE(true);
  }
  catch (...)
  {
  }
  // EXPECT_TRUE(false);
}

TEST(TestSuite, load_config)
{
  ros::NodeHandle nh;

  test_data_.config_loaded_ = test_data_.config_.loadConfig(nh);
  EXPECT_TRUE(test_data_.config_loaded_);
}

TEST(TestSuite, load_plugin)
{
  EXPECT_TRUE(test_data_.config_loaded_);

  auto& con_cfg = test_data_.config_.connections_[0];

  test_data_.pDriver_->loadPlugin(con_cfg, test_data_.cmd_reg_loader, test_data_.cmd_register_);

  try
  {
    TestData test_data_fail = test_data_;
    auto con_cfg_copy = con_cfg;

    con_cfg_copy.rmi_plugin_package_ = "invalid";
    test_data_.pDriver_->loadPlugin(con_cfg_copy, test_data_fail.cmd_reg_loader, test_data_fail.cmd_register_);

    FAIL() << "Expected an exception";
  }
  catch (...)
  {
  }

  EXPECT_TRUE(test_data_.cmd_register_->handlers().size() > 0);
}

TEST(TestSuite, test_plugin)
{
  EXPECT_TRUE(test_data_.config_loaded_);

  {  // required gets
    robot_movement_interface::Command cmd;
    cmd.command_type = "GET";

    cmd.pose_type = "JOINT_POSITION";
    auto cmd_handler = test_data_.cmd_register_->findHandler(cmd);
    EXPECT_TRUE(cmd_handler);

    cmd.pose_type = "TOOL_FRAME";
    cmd_handler = test_data_.cmd_register_->findHandler(cmd);
    EXPECT_TRUE(cmd_handler);

    cmd.pose_type = "VERSION";
    cmd_handler = test_data_.cmd_register_->findHandler(cmd);
    EXPECT_TRUE(cmd_handler);

    cmd.pose_type = "ERROR";
    cmd_handler = test_data_.cmd_register_->findHandler(cmd);
    EXPECT_FALSE(cmd_handler);
  }

  {  // ptp joints
    robot_movement_interface::Command cmd;
    cmd.command_type = "PTP";
    cmd.pose_type = "JOINTS";

    auto& conn = test_data_.config_.connections_[0];
    for (int i = 0; i < conn.joints_.size(); i++)
    {
      cmd.pose.push_back(i);
    }

    auto cmd_handler = test_data_.cmd_register_->findHandler(cmd);
    EXPECT_TRUE(cmd_handler);

    cmd.pose.push_back(42);
    cmd_handler = test_data_.cmd_register_->findHandler(cmd);
    EXPECT_FALSE(cmd_handler);
  }
}

TEST(TestSuite, test_connection)
{
  EXPECT_TRUE(test_data_.config_loaded_);

  auto& con_cfg = test_data_.config_.connections_[0];
  test_data_.connector_ =
      std::make_shared<Connector>(con_cfg.ns_, io_service_, con_cfg.ip_address_, con_cfg.port_, con_cfg.joints_,
                                  test_data_.cmd_reg_loader, test_data_.cmd_register_, true);

  auto& shared = test_data_.connector_;
  shared->connect();

  // std::this_thread::sleep_for(std::chrono::seconds(4));

  robot_movement_interface::Command cmd;
  cmd.command_type = "GET";

  cmd.pose_type = "JOINT_POSITION";
  auto cmd_handler = test_data_.cmd_register_->findHandler(cmd);
  EXPECT_TRUE(cmd_handler);

  auto robot_cmd = cmd_handler->processMsg(cmd);

  EXPECT_TRUE(robot_cmd != nullptr);

  auto res = shared->sendCommand(*robot_cmd);

  EXPECT_TRUE(res.length() > 1) << "Blah " << res;

  {
    RobotCommand rob_cmd(RobotCommand::CommandType::Cmd, "ping", "");
    res = shared->sendCommand(rob_cmd);

    EXPECT_EQ("pong", res);
  }
}

bool testQuat(tf2::Quaternion& quat1, tf2::Quaternion& quat2, double range)
{
  using namespace util;

  bool is_close = RotationUtils::approxEqual(quat1, quat2, 0.001);
  if (is_close)
    std::cout << quat1 << "== " << quat2 << std::endl;
  else
    std::cout << quat1 << "!= " << quat2 << std::endl;

  return is_close;
}

tf2::Quaternion quatFromZYZDeg(double Z, double Y, double ZZ)
{
  using namespace util;
  return RotationUtils::quatFromZYZ(degToRad(Z), degToRad(Y), degToRad(ZZ));
}

tf2::Quaternion setQuat(double x, double y, double z, double w)
{
  return tf2::Quaternion(x, y, z, w).normalize();
}

TEST(TestSuite, rotations)
{
  using namespace util;

  tf2::Quaternion quat_to_comp;  // Manually set this to stuff
  tf2::Quaternion quat;

  quat_to_comp = setQuat(1, 0, 0, 0);
  quat = quatFromZYZDeg(180.0, 180.0, 0.0);
  EXPECT_TRUE(testQuat(quat, quat_to_comp, 0.001));

  quat_to_comp = setQuat(1, 1, 0, 0);
  EXPECT_FALSE(testQuat(quat, quat_to_comp, 0.001));

  quat_to_comp = setQuat(0.985, 0.006, -0.112, -0.128);
  quat = quatFromZYZDeg(131.419, 160.437, -49.355);
  EXPECT_TRUE(testQuat(quat, quat_to_comp, 0.001));

  quat_to_comp = setQuat(0.885, 0.106, -0.112, -0.128);
  EXPECT_FALSE(testQuat(quat, quat_to_comp, 0.001));
}

TEST(TestSuite, DISABLED_test2)
{
  TestCommandRegister reg;

  robot_movement_interface::Command cmd;
  cmd.command_type = "TEST";
  auto chtest = CommandHandler::createHandler(cmd, [](const robot_movement_interface::Command& cmd_msg) {
    return std::make_shared<RobotCommand>(RobotCommand::CommandType::Cmd, cmd_msg.command_type, cmd_msg.pose_type);
  });

  reg.addHandler(std::move(chtest));

  EXPECT_TRUE(reg.findHandler(cmd));

  EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");  // some tests need ROS framework
  ros::Time::init();
  ros::Duration(1).sleep();  // Sleep to allow rqt_console to detect the new node

  io_service_thread_ = std::thread([&]() { io_service_.run(); });
  test_data_.pDriver_.reset(new TestDriver());
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
