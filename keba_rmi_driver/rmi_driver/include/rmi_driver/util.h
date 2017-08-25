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
 *  Created on: Aug 25, 2017
 *      Author: Doug Smith
 */

#ifndef INCLUDE_RMI_DRIVER_UTIL_H_
#define INCLUDE_RMI_DRIVER_UTIL_H_

#include <ros/ros.h>
#include <string>
#include <vector>

namespace rmi_driver
{
namespace util
{
/**
 * \brief Convert a float into a string with no trailing zeroes
 *
 * @param fval float to convert
 * @param precision max number of decimals
 * @return string representation of fval
 */
std::string floatToStringNoTrailing(float fval, int precision);

///@{
/**
 * \brief Check if the sample is used, then check if msg matches sample.
 *
 * \details Checks that the sample vector is not empty, then checks if the size of the 2 vectors is equal.
 *
 * @todo This may not work now that I'm using more flexible criteria.  Maybe the sample should be a vector of sizes and
 * the sample's length can be checked against them?
 *
 * @param sample The stored sample to check.
 * @param msg The value from a message to check.
 * @return True if sample is used and not equal to msg.  This indicates a failure and the handler does not match.
 */
template <typename T>
bool usedAndNotEqual(const std::vector<T>& sample, const std::vector<T>& msg)
{
  return sample.size() > 0 && sample.size() != msg.size();
}

/**
 * \brief \copybrief usedAndNotEqual(const std::vector<T>& sample, const std::vector<T>& msg)
 *
 * \detail Checks that the sample is not empty and for exact equality.
 */
bool usedAndNotEqual(const std::string& sample, const std::string& msg);

///@}

}  // namespace util

}  // namespace rmi_driver

#endif /* INCLUDE_RMI_DRIVER_UTIL_H_ */
