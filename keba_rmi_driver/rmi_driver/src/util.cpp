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

#include "rmi_driver/util.h"

#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

namespace rmi_driver
{
namespace util
{
std::string floatToStringNoTrailing(float fval, int precision)
{
  // Convert the value to a string with fixed precision
  std::ostringstream oss;
  oss << std::setprecision(precision) << std::fixed;
  oss << fval;
  auto str = oss.str();
  // Remove extra trailing 0s or .
  boost::trim_right_if(str, boost::is_any_of("0"));
  boost::trim_right_if(str, boost::is_any_of("."));

  return str;
}

std::vector<double> stringToDoubleVec(const std::string& s)
{
  std::vector<double> doubleVec;
  std::vector<std::string> strVec;

  // Split the string at spaces into a vector of strings
  boost::split(strVec, s, boost::is_any_of(" "), boost::token_compress_on);

  // Insert the double value of each entry
  std::transform(strVec.begin(), strVec.end(), std::back_inserter(doubleVec),
                 [](const std::string& val) { return boost::lexical_cast<double>(val); });

  return doubleVec;
}

bool usedAndNotEqualIdx(int entry_index, const std::vector<float>& sample, const std::vector<float>& msg)
{
  // Check if the sample has anything
  if (sample.size() == 0)
    return false;

  // An invalid index was given.
  if (entry_index >= sample.size() || entry_index < 0)
    return true;

  // Get the value stored in sample[entry_index] and compare it with the size of msg
  auto expected_size = std::lround(sample[entry_index]);

  return msg.size() != expected_size;
}

bool usedAndNotEqual(const std::string& sample, const std::string& msg, int* entry_index)
{
  if (entry_index)
    *entry_index = 0;

  if (sample.length() <= 0)
    return false;

  boost::char_separator<char> sep("|", "", boost::keep_empty_tokens);
  boost::tokenizer<boost::char_separator<char>, std::string::const_iterator, std::string> tok(sample, sep);

  // Eclipse complains about auto&& with a tokenizer?
  for (const std::string& entry : tok)
  {
    // Found an entry that matches.
    if (entry.compare(msg) == 0)
      return false;

    if (entry_index)
      (*entry_index)++;
  }

  // A sample was given but no match was found.
  if (entry_index)
    *entry_index = 0;
  return true;
}

}  // namespace util

}  // namespace rmi_driver
