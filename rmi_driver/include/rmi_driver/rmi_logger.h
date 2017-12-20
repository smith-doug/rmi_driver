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

#include <boost/algorithm/string/replace.hpp>

// I was trying to get away from using defines, but the only way to get the __FILE__ etc info is from the preprocessor.
// Call these link member functions, logger_.DEBUG() << "blah";
#define DEBUG() DEBUG_(__FILE__, __LINE__, __ROSCONSOLE_FUNCTION__)
#define INFO() INFO_(__FILE__, __LINE__, __ROSCONSOLE_FUNCTION__)
#define WARN() WARN_(__FILE__, __LINE__, __ROSCONSOLE_FUNCTION__)
#define ERROR() ERROR_(__FILE__, __LINE__, __ROSCONSOLE_FUNCTION__)
#define FATAL() FATAL_(__FILE__, __LINE__, __ROSCONSOLE_FUNCTION__)

namespace rmi_driver
{
namespace rmi_log
{
class RmiLogger
{
public:
  typedef ros::console::levels::Level Level;

  RmiLogger(const std::string& module_name, const std::string& ns);

  class DebugEx
  {
  public:
    ros::console::LogLocation& loc_;

    DebugEx(const std::string& module, const std::string& ns, ros::console::LogLocation& loc,
            Level level = Level::Info);

    DebugEx(const std::string& module, const std::string& ns, ros::console::LogLocation& loc, Level level,
            const char* file, int line, const char* function);

    DebugEx(const DebugEx& other);

    DebugEx(DebugEx&& other);

    ~DebugEx();

    template <typename T>
    DebugEx& operator<<(const T& thing_to_log)
    {
      ss_ << thing_to_log;
      return *this;
    }

    // this is the type of std::cout
    typedef std::basic_ostream<char, std::char_traits<char> > CoutType;

    // this is the function signature of std::endl
    typedef CoutType& (*StandardEndLine)(CoutType&);

    // define an operator<< to take in std::endl
    //    DebugEx& operator<<(StandardEndLine manip)
    //    {
    //      // call the function, but we cannot return it's value
    //      manip(ss_);
    //
    //      return *this;
    //    }

    DebugEx& operator<<(std::ostream& (*f)(std::ostream&))
    {
      f(ss_);
      return *this;
    }

    // Doesn't really work since the state variables are created in the macro, all instances will share the same
    // throttle
    //    DebugEx& throttle(int rate)
    //    {
    //      throttle_ = rate;
    //      return *this;
    //    }

    /*template <typename... Args>
    void print(const char* fmt, Args const&... args)
    {
      ROS_ERROR(fmt, args...);
    }*/

  private:
    std::string module_name_;
    std::string ns_;
    std::stringstream ss_;
    Level level_;

    const char* file_;
    int line_;
    const char* function_;

    int throttle_ = 0;
  };

public:
  DebugEx DEBUG_(const char* file = 0, int line = 0, const char* function = 0)
  {
    return DebugEx(module_name_, ns_, log_location, Level::Debug, file, line, function);
  }

  DebugEx INFO_(const char* file = 0, int line = 0, const char* function = 0)
  {
    return DebugEx(module_name_, ns_, log_location, Level::Info, file, line, function);
  }

  DebugEx WARN_(const char* file = 0, int line = 0, const char* function = 0)
  {
    return DebugEx(module_name_, ns_, log_location, Level::Warn, file, line, function);
  }

  DebugEx ERROR_(const char* file = 0, int line = 0, const char* function = 0)
  {
    return DebugEx(module_name_, ns_, log_location, Level::Error, file, line, function);
  }

  DebugEx FATAL_(const char* file = 0, int line = 0, const char* function = 0)
  {
    return DebugEx(module_name_, ns_, log_location, Level::Error, file, line, function);
  }

  void setLoggerLevel(Level level);

  void disable()
  {
    // I can't find any other way to disable the output.  Count doesn't seem to work.
    setLoggerLevel(Level::Fatal);
  }

  /*std::string getName()
  {
    std::string ns_name = ns_;
    if (*ns_name.begin() == '/')
      ns_name.erase(0, 1);
    return module_name_ + ns_name;
  }*/

  std::string getName()
  {
    // std::string ns_name = boost::replace_all_copy(ns_, "/", "");
    // ns_name = ns_;
    return module_name_ + ":" + ns_;
    // return ns_name;
  }

private:
  std::string module_name_;
  std::string ns_;

  ::ros::console::LogLocation log_location;
  // bool enabled_;
};
}
}

#endif /* INCLUDE_RMI_DRIVER_RMI_LOGGER_H_ */
