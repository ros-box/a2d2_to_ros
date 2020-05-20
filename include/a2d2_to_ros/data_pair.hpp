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
#ifndef A2D2_TO_ROS__DATA_PAIR_HPP_
#define A2D2_TO_ROS__DATA_PAIR_HPP_

#include <std_msgs/Header.h>
#ifdef USE_FLOAT64
#include <std_msgs/Float64.h>
#else
#include <std_msgs/Float32.h>
#endif

namespace a2d2_to_ros {

/** @brief Convenience storage for timestamp/value pair. */
struct DataPair {
#ifdef USE_FLOAT64
  typedef std_msgs::Float64 value_type;
#else
  typedef std_msgs::Float32 value_type;
#endif
  const value_type value;
  const std_msgs::Header header;

  /**
   * @brief Factory method to build a DataPair.
   * @pre time is valid according to valid_ros_timestamp.
   */
  static DataPair build(double value, uint64_t time, std::string frame_id);

 private:
  DataPair(std_msgs::Header header, value_type value);
};  // struct DataPair

/**
 * @brief Compare two DataPair objects by timestamp.
 * @note Operator returns true iff lhs < rhs.
 */
struct DataPairTimeComparator {
  bool operator()(const DataPair& lhs, const DataPair& rhs) const;
};  // struct DataPairComparator

}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__DATA_PAIR_HPP_
