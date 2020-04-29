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
#else
#define X_WARN(s)
#define X_ERROR(s)
#endif

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace a2d2_to_ros {

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
 * @note This function has no test coverage. Its output is informative only.
 */
std::ostream& operator<<(std::ostream& os, Units u);

/**
 * @brief Convert a string unit name to enum.
 * @return An enum value of the string or UNKNOWN if no match is found.
 */
Units get_unit_enum(const std::string& unit_name);

/**
 * @brief Check that a value is appropriate for its unit type.
 *
 * null                     \in [-\infty, \infty]
 * Unit_PerCent             \in [0, 100]
 * Unit_DegreOfArc          \in [0, 360)
 * Unit_DegreOfArcPerSecon  \in [0, 360)
 * Unit_Bar                 \in [0, \infty)
 * Unit_KiloMeterPerHour    \in [0, \infty)
 * Unit_MeterPerSeconSquar  \in (-\infty, \infty)
 *
 * @return True iff the value is valid for the given units.
 */
template <typename T>
T validate_unit_value(Units u, const T& val) {
  constexpr auto ZERO = static_cast<T>(0.0);
  constexpr auto ONE_HUNDRED = static_cast<T>(100.0);
  constexpr auto MAX_DEGREE = static_cast<T>(360.0);
  constexpr auto INF = std::numeric_limits<T>::infinity();

  std::stringstream ss;
  ss << "Value invalid for units of type '" << u << "': " << val << " \\notin ";

  switch (u) {
    case Units::null: {
      const auto inside_min = (val >= -INF);
      const auto inside_max = (val <= INF);
      const auto valid = (inside_min && inside_max);
      if (!valid) {
        X_WARN(ss.str() << "[" << -INF << ", " << INF << "]");
      }
      return valid;
    }
    case Units::Unit_PerCent: {
      const auto inside_min = (val >= ZERO);
      const auto inside_max = (val <= ONE_HUNDRED);
      const auto valid = (inside_min && inside_max);
      if (!valid) {
        X_WARN(ss.str() << "[" << ZERO << ", " << ONE_HUNDRED << "]");
      }
      return valid;
    };
    case Units::Unit_DegreOfArc:
    case Units::Unit_DegreOfArcPerSecon: {
      const auto inside_min = (val >= ZERO);
      const auto inside_max = (val < MAX_DEGREE);
      const auto valid = (inside_min && inside_max);
      if (!valid) {
        X_WARN(ss.str() << "[" << ZERO << ", " << MAX_DEGREE << ")");
      }
      return valid;
    };
    case Units::Unit_Bar:
    case Units::Unit_KiloMeterPerHour: {
      const auto inside_min = (val >= ZERO);
      const auto inside_max = (val < INF);
      const auto valid = (inside_min && inside_max);
      if (!valid) {
        X_WARN(ss.str() << "[" << ZERO << ", " << INF << ")");
      }
      return valid;
    };
    case Units::Unit_MeterPerSeconSquar: {
      const auto inside_min = (val > -INF);
      const auto inside_max = (val < INF);
      const auto valid = (inside_min && inside_max);
      if (!valid) {
        X_WARN(ss.str() << "(" << -INF << ", " << INF << ")");
      }
      return valid;
    };
    default:
      X_ERROR("Unrecognized unit type '"
              << u << "', cannot validate. Returning false.");
      return false;
  }
}

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
