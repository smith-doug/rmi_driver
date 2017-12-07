/*
 * Copyright (c) 2017, Doug Smith, KEBA Corp
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
std::string floatToStringNoTrailing(float fval, int precision = 4);

/**
 * Convert a string of numbers separated by spaces into a vector of doubles.
 * @param s string of numbers.  "1 2 3.53 56.563"
 * @return Vector of doubles {1, 2, 3.53, 56.563}
 */
std::vector<double> stringToDoubleVec(const std::string& s);

/**
 * \brief Outputs a vector as a nice string.
 *
 * This will take a vector like [1,2,3] and output a string "{1, 2, 3}"
 * @param vec Vector of type T
 * @return formatted string
 */
template <typename T>
std::string vecToString(const std::vector<T>& vec)
{
  std::ostringstream oss;
  oss << "{";
  if (!vec.empty())
  {
    std::copy(vec.begin(), vec.end() - 1, std::ostream_iterator<T>(oss, ", "));
    oss << vec.back();
  }
  oss << "}";

  return oss.str();
}

///@{
/*
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
// template <typename T>
// bool usedAndNotEqual(const std::vector<T>& sample, const std::vector<T>& msg)
//{
//  return sample.size() > 0 && sample.size() != msg.size();
//}

/**
 * \brief Check if a vector is used and if the number of entries matches the number stored in the same at position
 * entry_index.
 *
 * This will check if sample.size() > 0 and lround(sample[entry_index]) == msg.size().
 * @param entry_index Index of sample to check.
 * @param sample Stored sample vector
 * @param msg vector from the message
 * @return True if sample is used and the size is not equal.  This indicates a failure and the handler does not match.
 */
bool usedAndNotEqualIdx(int entry_index, const std::vector<float>& sample, const std::vector<float>& msg);

/**
 * \brief Check if the sample is used, then check if msg matches sample.  Works with multiple sample entries.
 *
 * \details This will first check if the length of sample is > 0 to indicate that it is used.  It will then compare the
 * value/values in sample with msg.  Multiple sample values can be entered by separating them with | .  Like
 * "JOINTS|QUATERNION|XYZ".  If a pointer to entry_index is provided, it will give the index of the entry that was
 * found.
 * @param sample Stored sample.  Can be 1 entry like "JOINTS" or multiple entries like "JOINTS|QUATERNION"
 * @param msg The received value
 * @param entry_index Optional.  Store the index of the entry that was found.
 *
 * @return True if the message disqualifies this as a match.
 */
bool usedAndNotEqual(const std::string& sample, const std::string& msg, int* entry_index = 0);

/**
 * \brief Sort a vector of type Tsource by a vector of indices
 *
 * ret[n] = data[indices[n]];
 *
 * indices = {1, 2, 0}
 * data = {123, 456, 789}
 * sorted = {456, 789, 123}
 *
 * @param indices Indicates the order to rearrange the data by.
 * @param data The data to sort
 * @return Sorted vector.  Tsource must be able to be placed in Tdest.
 */
template <typename Tdest, typename Tsource>
std::vector<Tdest> sortVectorByIndices(const std::vector<size_t>& indices, const std::vector<Tsource>& data)
{
  std::vector<Tdest> ret;
  if (data.size() == 0)  // data isn't used
    return ret;

  if (indices.size() != data.size())  // sizes have to be equal to sort
  {
    std::stringstream ss;
    ss << "sortVector failed: indices.size(" << indices.size() << ") != data.size(" << data.size() << ")";
    throw std::runtime_error(ss.str());
  }

  ret.reserve(indices.size());

  std::transform(indices.begin(), indices.end(), std::back_inserter(ret), [&](size_t i) { return data[i]; });

  return ret;
}

///@}

}  // namespace util

}  // namespace rmi_driver

#endif /* INCLUDE_RMI_DRIVER_UTIL_H_ */
