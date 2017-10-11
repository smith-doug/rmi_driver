/*#include "industrial_utils/param_utils.h"
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

#include "rmi_driver/rmi_config.h"
#include <XmlRpcValue.h>

namespace rmi_driver
{
bool getListParamRmi(const std::string param_name, std::vector<DriverConfig::ConnectionConfig>& list_param)
{
  XmlRpc::XmlRpcValue rpc_list;

  list_param.clear();  // clear out return value

  if (!ros::param::get(param_name, rpc_list))
  {
    ROS_ERROR_STREAM("Failed to get parameter: " << param_name);
    return false;
  }

  if (rpc_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " not of list type");
    return false;
  }

  for (int i = 0; i < rpc_list.size(); ++i)
  {
    DriverConfig::ConnectionConfig map;
    if (!map.parse(rpc_list[i]))
    {
      ROS_ERROR_STREAM("Failed to parse parameter: " << param_name << "[" << i << "]");
      return false;
    }

    list_param.push_back(map);
  }

  return true;
}

void DriverConfig::loadConfig(ros::NodeHandle& nh)
{
  ROS_INFO_STREAM(__func__ << " loading");

  // Load "global" params first
  loadParam(nh, "/rmi_driver/publish_rate", publishing_rate_, 30);

  // Load connection specific params
  //  ConnectionConfig cfg;
  //
  //  loadParam(nh, "/rmi_driver/connection/ip_address", cfg.ip_address_, "192.168.100.100");
  //
  //  loadParam(nh, "/rmi_driver/connection/port", cfg.port_, 30000);
  //
  //  loadParam(nh, "/rmi_driver/connection/rmi_plugin_package", cfg.rmi_plugin_package_, "keba_rmi_plugin");
  //
  //  loadParam(nh, "/rmi_driver/connection/rmi_plugin_lookup_name", cfg.rmi_plugin_lookup_name_, "keba_rmi_plugin::"
  //                                                                                              "KebaCommandRegister");
  //
  //  connections_.push_back(cfg);

  std::string config_name = "rmi_driver_map";
  bool sadas = getListParamRmi(config_name, connections_);
  return;
}
bool DriverConfig::ConnectionConfig::parse(XmlRpc::XmlRpcValue& value)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("JointGroupMap not struct type");
    return false;
  }

  std::string key = "connection";
  if (!value.hasMember(key) || (value[key].getType() != XmlRpc::XmlRpcValue::TypeInt))
  {
    ROS_ERROR("JointGroupMap 'group' field missing or invalid type");
    return false;
  }
  this->connection_ = static_cast<int>(value[key]);

  key = "ns";
  if (!value.hasMember(key) || (value[key].getType() != XmlRpc::XmlRpcValue::TypeString))
  {
    ROS_ERROR("JointGroupMap 'ns' field missing or invalid type");
    return false;
  }
  this->ns_ = static_cast<std::string>(value[key]);

  key = "ip_address";
  if (!value.hasMember(key) || (value[key].getType() != XmlRpc::XmlRpcValue::TypeString))
  {
    ROS_ERROR("JointGroupMap 'ns' field missing or invalid type");
    return false;
  }
  this->ip_address_ = static_cast<std::string>(value[key]);

  key = "port";
  if (!value.hasMember(key) || (value[key].getType() != XmlRpc::XmlRpcValue::TypeInt))
  {
    ROS_ERROR("JointGroupMap 'group' field missing or invalid type");
    return false;
  }
  this->port_ = static_cast<int>(value[key]);

  key = "rmi_plugin_package";
  if (!value.hasMember(key) || (value[key].getType() != XmlRpc::XmlRpcValue::TypeString))
  {
    ROS_ERROR("JointGroupMap 'ns' field missing or invalid type");
    return false;
  }
  this->rmi_plugin_package_ = static_cast<std::string>(value[key]);

  key = "rmi_plugin_lookup_name";
  if (!value.hasMember(key) || (value[key].getType() != XmlRpc::XmlRpcValue::TypeString))
  {
    ROS_ERROR("JointGroupMap 'ns' field missing or invalid type");
    return false;
  }
  this->rmi_plugin_lookup_name_ = static_cast<std::string>(value[key]);

  key = "joints";
  if (!value.hasMember(key) || !industrial_utils::param::getListParam(value[key], this->joints_))
  {
    ROS_ERROR("JointGroupMap 'joints' field missing or invalid type");
    return false;
  }

  return true;
}

}  // namespace rmi_driver
