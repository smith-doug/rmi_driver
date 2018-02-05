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
 * \brief Create rotations for different Euler rotation orders
 *
 * Transformation formulas from "Introduction to Robotics"
 */
class RotationUtils
{
public:
  /**
   * \brief Create a rotation matrix rotated around Z
   * @param th angle in radians
   * @return Rotation matrix
   */
  static tf2::Matrix3x3 rotZ(double th);

  /**
   * \brief Create a rotation matrix rotated around Y
   * @param th angle in radians
   * @return Rotation matrix
   */
  static tf2::Matrix3x3 rotY(double th);

  /**
   * \brief Create a rotation matrix rotated around X
   * @param th angle in radians
   * @return Rotation matrix
   */
  static tf2::Matrix3x3 rotX(double th);

  /**
   * \brief Create a rotation matrix from Euler ZYZ' angles
   * @param Z Z rotation in radians
   * @param Y Y rotation in radians
   * @param ZZ Z' rotation in radians
   * @return Rotation matrix equivalent to ZYZ'
   */
  static tf2::Matrix3x3 rotZYZ(const tf2Scalar& Z, const tf2Scalar& Y, const tf2Scalar& ZZ);

  /**
   * \brief Create a normalized Quaternion from Euler ZYZ' angles
   * @param Z Z rotation in radians
   * @param Y Y rotation in radians
   * @param ZZ Z' rotation in radians
   * @return Quaternion equivalent to ZYZ'
   */
  static tf2::Quaternion quatFromZYZ(const tf2Scalar& Z, const tf2Scalar& Y, const tf2Scalar& ZZ);

  /**
   * \brief Check if 2 quaternions are approximately equal.
   *
   * The quaternions will be normalized before comparing.
   * @param quat1 Quaternion 1
   * @param quat2 Quaternion 2
   * @param range How close they have to be
   * @return True if they are close
   */
  static bool approxEqual(const tf2::Quaternion& quat1, const tf2::Quaternion& quat2, double range = 0.001);

  /**
   * \brief Format the quaternion as a nice string
   * @param quat the quaternion
   * @return formatted string
   */
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
