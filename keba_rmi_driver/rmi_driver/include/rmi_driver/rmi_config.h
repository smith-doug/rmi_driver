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
 *  Created on: Oct 11, 2017
 *      Author: Doug Smith
 */

/*
 * The code for handling XmlRpcValues is based on code from industrial_core/industrial_utils.  The license for that
 * project is as follows:
 * */

/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2012, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*       * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*       * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*       * Neither the name of the Southwest Research Institute, nor the names
*       of its contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \brief Load configuration data from yaml files.
 *
 * Note: XmlRpc loading code is based on code from the industrial_core/industrial_utils project.
 */

#ifndef INCLUDE_RMI_DRIVER_RMI_CONFIG_H_
#define INCLUDE_RMI_DRIVER_RMI_CONFIG_H_

#include <XmlRpcValue.h>

#include <ros/ros.h>
#include <map>
#include <string>
#include <vector>

namespace rmi_driver
{
class ConnectionConfig
{
public:
  ConnectionConfig()
  {
  }

  int connection_ = 0;                  /// Connection number (currently unused)
  std::string ns_;                      /// Namespace for this connection
  std::string ip_address_;              /// IP of this connection
  int port_ = 0;                        /// Port number of this connection
  std::string rmi_plugin_package_;      /// Package name the plugin lives in
  std::string rmi_plugin_lookup_name_;  /// The actual class name that is exported
  std::vector<std::string> joints_;     /// List of joints

  /**
   * \brief Load the settings for this connection
   *
   * Based on industrial_utils
   *
   * @param value The TypeStruct data for this connection
   * @return True if OK
   */
  bool parse(XmlRpc::XmlRpcValue &value);
};

class DriverConfig
{
public:
  DriverConfig() : publishing_rate_(30)
  {
  }

  // Force template deduction to be a string and not a char[] when passed a "" string
  template <typename T>
  struct identity
  {
    typedef T type;
  };

  template <typename T>
  void loadParam(ros::NodeHandle &nh, const std::string &key, T &val, const typename identity<T>::type &def)
  {
    std::ostringstream oss;
    bool loadOk = nh.param<T>(key, val, def);
    if (loadOk)
      oss << "Loaded param: " << key << ".  Value: " << val;
    else
      oss << "Failed to load: " << key << ". Using default value: " << def;

    ROS_INFO_STREAM(oss.str());
  }

  /**
   * \brief load the yaml file parameters.  Loads the global parameters then each connection.
   * @param nh The driver node
   * @return True if all the configs loaded ok
   */
  bool loadConfig(ros::NodeHandle &nh);

  std::vector<ConnectionConfig> connections_;  /// All the connections

  int publishing_rate_;
};

/**
 * \brief Gets parameter list as vector of strings
 *
 * Original from industrial_utils
 *
 * \param value contents of parameter value
 * \param list_param populated with parameter value(s)
 *
 * \return true if parameter
 */
bool getListParam(XmlRpc::XmlRpcValue rpc_list, std::vector<std::string> &list_param);

/**
 * \brief Loads all of the ConnectionConfigs
 *
 * Original from industrial_utils
 *
 * @param param_name Name of the root element
 * @param list_param Vector of the ConnectionConfigs
 * @return True if ok
 */
bool getListParam(const std::string param_name, std::vector<ConnectionConfig> &list_param);

}  // namespace rmi_driver

#endif /* INCLUDE_RMI_DRIVER_RMI_CONFIG_H_ */
