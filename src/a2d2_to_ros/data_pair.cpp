/*
 * MIT License
 *
 * Copyright (c) 2020 Mapless AI, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#include "a2d2_to_ros/data_pair.hpp"

#include "a2d2_to_ros/conversions.hpp"

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

DataPair::DataPair(std_msgs::Header header, value_type value)
    : header(std::move(header)), value(std::move(value)) {}

//------------------------------------------------------------------------------

bool DataPairTimeComparator::operator()(const DataPair& lhs,
                                        const DataPair& rhs) const {
  return (lhs.header.stamp < rhs.header.stamp);
}

//------------------------------------------------------------------------------

DataPair DataPair::build(double value, uint64_t time, std::string frame_id) {
  std_msgs::Header header;
  header.seq = 0;
  header.frame_id = std::move(frame_id);
  header.stamp = a2d2_timestamp_to_ros_time(time);

  value_type val;
  val.data = value;

  return DataPair(header, val);
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros
