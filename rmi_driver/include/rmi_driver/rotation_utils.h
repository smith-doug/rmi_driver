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
 *  Created on: Feb 1, 2018
 *      Author: Doug Smith
 */

#ifndef INCLUDE_RMI_DRIVER_ROTATION_UTILS_H_
#define INCLUDE_RMI_DRIVER_ROTATION_UTILS_H_

#include "rmi_driver/util.h"

#include <tf2/LinearMath/Matrix3x3.h>

namespace rmi_driver
{
namespace util
{
/**
 *
 *
 * Transformation formulas from "Introduction to Robotics"
 */
class RotationUtils
{
public:
  static tf2::Matrix3x3 rotZ(double th);
  static tf2::Matrix3x3 rotY(double th);
  static tf2::Matrix3x3 rotX(double th);

  static tf2::Matrix3x3 rotZYZ(const tf2Scalar& Z, const tf2Scalar& Y, const tf2Scalar& ZZ);

  static tf2::Quaternion quatFromZYZ(const tf2Scalar& Z, const tf2Scalar& Y, const tf2Scalar& ZZ);

  static bool approxEqual(tf2::Quaternion quat1, tf2::Quaternion quat2, double range = 0.01);

  static std::string quatToString(const tf2::Quaternion& quat);
};

inline std::ostream& operator<<(std::ostream& o, const tf2::Quaternion& quat)
{
  auto quat_str = RotationUtils::quatToString(quat);
  o << quat_str;
  return o;
}

}  // namespace util
} /* namespace rmi_driver */

#endif /* INCLUDE_RMI_DRIVER_ROTATION_UTILS_H_ */
