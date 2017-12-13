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

// based on https://stackoverflow.com/a/23482759

#ifndef INCLUDE_RMI_DRIVER_RMI_LOGGER_H_
#define INCLUDE_RMI_DRIVER_RMI_LOGGER_H_

#include <ros/ros.h>

#include <ostream>
#include <sstream>
#include <string>

namespace rmi_driver
{
namespace rmi_log
{
class RmiLogger
{
public:
  RmiLogger(const std::string& module_name, const std::string& ns) : module_name_(module_name), ns_(ns)
  {
  }
  ~RmiLogger()
  {
  }

  typedef ros::console::levels::Level Level;

  class DebugEx
  {
  private:
    std::string module_name_;
    std::string ns_;
    std::stringstream ss_;
    Level level_;

  public:
    DebugEx(const std::string& module, const std::string& ns, Level level = Level::Info)
      : module_name_(module), ns_(ns), level_(level)
    {
    }
    DebugEx(const DebugEx& other) : module_name_(other.module_name_), ns_(other.ns_), level_(other.level_)
    {
    }
    ~DebugEx()
    {
      std::string str = ss_.str();
      if (!str.empty())
      {
        switch (level_)
        {
          case Level::Debug:
            ROS_DEBUG_STREAM_NAMED(module_name_, module_name_ << ":" << ns_ << str);
            break;
          case Level::Info:
            ROS_INFO_STREAM_NAMED(module_name_, module_name_ << ":" << ns_ << str);
            break;
          case Level::Warn:
            ROS_WARN_STREAM_NAMED(module_name_, module_name_ << ":" << ns_ << str);
            break;
          case Level::Error:
            ROS_ERROR_STREAM_NAMED(module_name_, module_name_ << ":" << ns_ << str);
            break;
          case Level::Fatal:
            ROS_FATAL_STREAM_NAMED(module_name_, module_name_ << ":" << ns_ << str);
            break;
        }
      }
    }
    template <typename T>
    DebugEx& operator<<(const T& thing_to_log)
    {
      ss_ << thing_to_log;
      return *this;
    }
  };

public:
  DebugEx DEBUG()
  {
    return DebugEx(module_name_, ns_, Level::Debug);
  }

  DebugEx INFO()
  {
    return DebugEx(module_name_, ns_, Level::Info);
  }

  DebugEx WARN()
  {
    return DebugEx(module_name_, ns_, Level::Warn);
  }

  DebugEx ERROR()
  {
    return DebugEx(module_name_, ns_, Level::Error);
  }

  void setLoggerLevel(Level level)
  {
    std::string log_name = std::string(ROSCONSOLE_NAME_PREFIX) + "." + module_name_;
    if (ros::console::set_logger_level(log_name, level))
      ros::console::notifyLoggerLevelsChanged();
  }

  void disable()
  {
    // I can't find any other way to disable the output
    setLoggerLevel(Level::Fatal);
  }

private:
  std::string module_name_;
  std::string ns_;
};
}
}

#endif /* INCLUDE_RMI_DRIVER_RMI_LOGGER_H_ */
