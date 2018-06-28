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

#include <rmi_driver/rotation_utils.h>

namespace rmi_driver
{
namespace util
{
tf2::Matrix3x3 RotationUtils::rotZ(double th)
{
  tf2Scalar cth(tf2Cos(th));
  tf2Scalar sth(tf2Sin(th));

  // clang-format off
  return tf2::Matrix3x3(cth,  -sth,   0,
                        sth,  cth,    0,
                        0,    0,      1);
  // clang-format on
}

tf2::Matrix3x3 RotationUtils::rotY(double th)
{
  tf2Scalar cth(tf2Cos(th));
  tf2Scalar sth(tf2Sin(th));

  // clang-format off
  return tf2::Matrix3x3(cth,  0,  sth,
                        0,    1,  0,
                        -sth, 0,  cth);
  // clang-format on
}

tf2::Matrix3x3 RotationUtils::rotX(double th)
{
  tf2Scalar cth(tf2Cos(th));
  tf2Scalar sth(tf2Sin(th));

  // clang-format off
  return tf2::Matrix3x3(1,  0,    0,
                        0,  cth,  -sth,
                        0,  sth,  cth);
  // clang-format on
}

tf2::Matrix3x3 RotationUtils::rotZYZ(const tf2Scalar& Z, const tf2Scalar& Y, const tf2Scalar& ZZ)
{
  auto rot = RotationUtils::rotZ(Z) * RotationUtils::rotY(Y) * RotationUtils::rotZ(ZZ);
  return rot;
}

tf2::Quaternion RotationUtils::quatFromZYZ(const tf2Scalar& Z, const tf2Scalar& Y, const tf2Scalar& ZZ)
{
  auto rot = RotationUtils::rotZYZ(Z, Y, ZZ);
  tf2::Quaternion quat;
  rot.getRotation(quat);
  return quat.normalize();
}

bool RotationUtils::approxEqual(const tf2::Quaternion& quat1, const tf2::Quaternion& quat2, double range)
{
  // https://answers.unity.com/questions/288338/how-do-i-compare-quaternions.html
  auto quat1_norm = quat1.normalized();
  auto quat2_norm = quat2.normalized();

  return quat1_norm.dot(quat2_norm) > 1 - range;
}

std::string RotationUtils::quatToString(const tf2::Quaternion& quat)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(4) << "[X:" << quat.getX() << " Y:" << quat.getY() << " Z:" << quat.getZ()
     << " W:" << quat.getW() << "]";

  return ss.str();
}

}  // namespace util
} /* namespace rmi_driver */
