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
#ifndef A2D2_TO_ROS__LIB_A2D2_TO_ROS_HPP_
#define A2D2_TO_ROS__LIB_A2D2_TO_ROS_HPP_

#ifdef _ENABLE_A2D2_ROS_LOGGING_
#include <ros/console.h>
#define X_WARN(s) ROS_WARN_STREAM(s)
#define X_ERROR(s) ROS_ERROR_STREAM(s)
#define X_FATAL(s) ROS_FATAL_STREAM(s)
#else
#define X_WARN(s)
#define X_ERROR(s)
#define X_FATAL(s)
#endif

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

namespace a2d2_to_ros {

/** @brief Convenience storage for timestamp/value pair. */
struct DataPair {
  const std_msgs::Header header;
  const std_msgs::Float64 value;

  /**
   * @brief Factory method to build a DataPair.
   * @pre time is valid according to valid_ros_timestamp.
   */
  static DataPair build(double value, uint64_t time, std::string frame_id);

 private:
  DataPair(std_msgs::Header header, std_msgs::Float64 value);
};  // struct DataPair

/**
 * @brief Compare two DataPair objects by timestamp.
 * @note Operator returns true iff lhs < rhs.
 */
struct DataPairTimeComparator {
  bool operator()(const DataPair& lhs, const DataPair& rhs) const;
};  // struct DataPairComparator

/**
 * @brief Test whether a given timestamp is covertible to ROS time.
 * @return True iff converting time to ros::Time will not cause overflow.
 */
bool valid_ros_timestamp(uint64_t time);

/**
 * @brief Convert Unix EPOCH milisecond stamps to ROS times.
 * @pre Integer timestamp in microseconds.
 * @pre time is valid according to valid_a2s2_timestamp
 */
ros::Time a2d2_timestamp_to_ros_time(uint64_t time);

/**
 * @brief Load an entire JSON file into memory as a string.
 * @pre The file pointed to by path is valid JSON.
 * @note This function has no test coverage.
 * @note
 * @return The json text, or an empty string if loading the file failed.
 */
std::string get_json_file_as_string(const std::string& path);

/**
 * @brief Unit in the unlabeled A2D2 data set.
 * @note Some names are missing letters, but that's how it is in the data set.
 */
enum class Units {
  null = 0,
  Unit_Bar = 1,
  Unit_PerCent = 2,
  Unit_DegreOfArc = 3,
  Unit_KiloMeterPerHour = 4,
  Unit_MeterPerSeconSquar = 5,
  Unit_DegreOfArcPerSecon = 6,
  UNKNOWN = 7
};

/**
 * @brief Print name of Units
 * @note This function has no test coverage.
 */
std::ostream& operator<<(std::ostream& os, Units u);

/**
 * @brief Convert a string unit name to enum.
 * @return An enum value of the string or UNKNOWN if no match is found.
 */
Units get_unit_enum(const std::string& unit_name);

/**
 * @brief Convert data set units to ROS units.
 * @pre validate_unit_value returns true for the given unit/value pair.
 * @note See: https://www.ros.org/reps/rep-0103.html
 * @note Unit_PerCent values [0, 100] are projected to [0, 1].
 * @note Unit_null and Unit_Bar are passed through unmodified.
 * @return val in ROS standard units, or NaN if there is no conversion. Return
 * value is undefined if pre-condition is not satisfied.
 */
template <typename T>
T to_ros_units(Units u, const T& val) {
  switch (u) {
    case Units::Unit_DegreOfArc:
    case Units::Unit_DegreOfArcPerSecon:
      return (val * static_cast<T>(M_PI / 180.0));
      break;
    case Units::Unit_KiloMeterPerHour:
      return (val * static_cast<T>(1000.0 / 60.0));
      break;
    case Units::Unit_PerCent:
      return (val / static_cast<T>(100.0));
      break;
    case Units::null:
    case Units::Unit_Bar:
    case Units::Unit_MeterPerSeconSquar:
      return val;
      break;
    default:
      X_ERROR("Unrecognized unit enum value '"
              << u << "', cannot convert. Returning NaN.");
      return std::numeric_limits<T>::quiet_NaN();
      break;
  }
}

}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__LIB_A2D2_TO_ROS_HPP_
