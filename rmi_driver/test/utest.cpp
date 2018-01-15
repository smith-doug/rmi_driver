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

#include <memory>

#include <rmi_driver/commands.h>
#include <rmi_driver/connector.h>
#include <rmi_driver/driver.h>

using namespace rmi_driver;

class TestData
{
public:
  DriverConfig config_;
  /// The CommandRegister that was loaded by the plugin
  CommandRegisterPtr cmd_register_;

  /// "The ClassLoader must not go out scope while you are using the plugin."  Keep it alive.
  CmhLoaderPtr cmh_loader_;
};
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

TestDriver* pDriver_;

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

TEST(TestSuite, driver_connect)
{
  boost::asio::io_service io_service_;

  std::vector<std::string> joint_names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                           "wrist_2_joint",      "wrist_3_joint",       "rail_to_base" };

  //  auto shared = std::make_shared<Connector>("/", io_service_, "127.0.0.1", 30000, joint_names, commands, cmh_loader,
  //                                            config_.clear_commands_on_error_);

  ros::NodeHandle nh;
  try
  {
    pDriver_->start();

    auto& conn_map = pDriver_->getConnMap();
    auto conn_ptr = conn_map.begin()->second;

    robot_movement_interface::Command cmd;
    cmd.command_type = "WAIT";
    cmd.pose_type = "IS_FINISHED";
    auto cmh = conn_ptr->getCommandRegister()->findHandler(cmd);
    EXPECT_TRUE(cmh);

    cmd.command_type = "INVALID";
    cmh = conn_ptr->getCommandRegister()->findHandler(cmd);
    EXPECT_FALSE(cmh);

    // Driver driver_;
    // driver_.start();
    /*
    auto& map = driver_.getConnMap();
    auto conn = map.begin()->second;

    robot_movement_interface::Command cmd;
    cmd.command_type = "WAIT";
    cmd.pose_type = "IS_FINISHED";
    auto cmh = conn->getCommandRegister()->findHandler(cmd);
    EXPECT_TRUE(cmh);

    cmd.command_type = "INVALID";
    cmh = conn->getCommandRegister()->findHandler(cmd);
    EXPECT_FALSE(cmh);

    // driver_.auto &conn = *driver_.conn_map_.begin()->second;
  */
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

  EXPECT_TRUE(test_data_.config_.loadConfig(nh));
}

TEST(TestSuite, load_plugin)
{
  EXPECT_TRUE(test_data_.config_.is_loaded_);

  auto& con_cfg = test_data_.config_.connections_[0];
  test_data_.cmh_loader_.reset(new CmhLoader(con_cfg.rmi_plugin_package_, "rmi_driver::"
                                                                          "CommandRegister"));

  test_data_.cmd_register_ = test_data_.cmh_loader_->createUniqueInstance(con_cfg.rmi_plugin_lookup_name_);
  test_data_.cmd_register_->initialize(con_cfg.joints_);
  EXPECT_TRUE(test_data_.cmd_register_->handlers().size() > 0);
}

TEST(TestSuite, test_plugin)
{
  EXPECT_TRUE(test_data_.config_.is_loaded_);

  {
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

  {
    robot_movement_interface::Command cmd;
  }
}

TEST(TestSuite, test2)
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

  pDriver_ = new TestDriver();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
