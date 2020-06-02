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
#ifndef A2D2_TO_ROS__CONVERSIONS_HPP_
#define A2D2_TO_ROS__CONVERSIONS_HPP_

#include <ros/time.h>

#include <cstdint>
#include <cstdlib>

namespace a2d2_to_ros {

/**
 * @brief Sensor data is recorded in TAI, bus signal data in UTC. This function
 * converts TAI to UTC for all times in the dataset so that we can unify time
 * representations.
 *
 * @pre mu_s is in microseconds
 * @pre mu_s is in TAI
 * @pre mu_s is a timestamp from the data
 *
 * @note THIS IS NOT FOR GENERAL USE, ONLY WITH THE A2D2 DATASET. The delta
 * between TAI and UTC changes every leap year, so this function is only valid
 * for a specific range of time.
 */
uint64_t TAI_to_UTC(uint64_t mu_s);

/**
 * @brief Convert a 2D index to a 1D index according to CNPY convention.
 */
size_t flatten_2d_index(size_t width, size_t row, size_t col);

/**
 * @brief Convert Unix EPOCH milisecond stamps to ROS times.
 * @pre Integer timestamp in microseconds.
 * @pre time is valid according to valid_a2s2_timestamp
 */
ros::Time a2d2_timestamp_to_ros_time(uint64_t time);

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
T to_ros_units(const std::string& unit_name, const T& val) {
  const auto u = get_unit_enum(unit_name);
  switch (u) {
    case Units::Unit_DegreOfArc:
    case Units::Unit_DegreOfArcPerSecon:
      return (val * static_cast<T>(M_PI / 180.0));
      break;
    case Units::Unit_KiloMeterPerHour:
      return (val * static_cast<T>(1000.0 / (60.0 * 60.0)));
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
      return std::numeric_limits<T>::quiet_NaN();
      break;
  }
}

}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__CONVERSIONS_HPP_
