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
 *  Created on: Dec 13, 2017
 *      Author: Doug Smith
 */

#include "rmi_driver/rmi_logger.h"

namespace rmi_driver
{
namespace rmi_log
{
RmiLogger::RmiLogger(const std::string& module_name, const std::string& ns) : module_name_(module_name), ns_(ns)
{
}

void RmiLogger::setLoggerLevel(Level level)
{
  std::string log_name = std::string(ROSCONSOLE_NAME_PREFIX) + "." + module_name_;
  if (ros::console::set_logger_level(log_name, level))
    ros::console::notifyLoggerLevelsChanged();
}

RmiLogger::DebugEx::DebugEx(const std::string& module, const std::string& ns, Level level)
  : module_name_(module), ns_(ns), level_(level)
{
}

RmiLogger::DebugEx::DebugEx(const DebugEx& other)
  : module_name_(other.module_name_), ns_(other.ns_), level_(other.level_)
{
}

RmiLogger::DebugEx::DebugEx(DebugEx&& other)
  : module_name_(std::move(other.module_name_)), ns_(std::move(other.ns_)), level_(other.level_)
{
}

RmiLogger::DebugEx::~DebugEx()
{
  std::string str = ss_.str();
  if (!str.empty())
  {
    //    auto log_name = getName();
    //    if (throttle_ > 0)
    //    {
    //      ROS_ERROR_STREAM_THROTTLE_NAMED(throttle_, log_name, log_name << " " << str);
    //      return;
    //    }

    switch (level_)
    {
      case Level::Debug:
        ROS_DEBUG_STREAM_NAMED(getName(), module_name_ << ":" << ns_ << " " << str);
        break;
      case Level::Info:
        ROS_INFO_STREAM_NAMED(getName(), module_name_ << ":" << ns_ << " " << str);
        break;
      case Level::Warn:
        ROS_WARN_STREAM_NAMED(getName(), module_name_ << ":" << ns_ << " " << str);
        break;
      case Level::Error:
        ROS_ERROR_STREAM_NAMED(getName(), module_name_ << ":" << ns_ << " " << str);
        break;
      case Level::Fatal:
        ROS_FATAL_STREAM_NAMED(getName(), module_name_ << ":" << ns_ << " " << str);
        break;
    }
  }
}

}  // namespace log

}  // namespace rmi_driver
