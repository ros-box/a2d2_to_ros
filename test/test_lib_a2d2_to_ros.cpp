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
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

// uncomment this define to log warnings and errors
//#define _ENABLE_A2D2_ROS_LOGGING_
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"

static constexpr auto INF = std::numeric_limits<double>::infinity();
static constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();
static constexpr auto ONE_MILLION = static_cast<uint64_t>(1000000);
static constexpr auto ONE_THOUSAND = static_cast<uint64_t>(1000);

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, a2d2_timestamp_to_ros_time) {
  constexpr auto TIME = static_cast<uint64_t>(1554122338652775);
  const auto ros_time = a2d2_timestamp_to_ros_time(TIME);
  const auto nsecs = ros_time.toNSec();
  const auto test_secs = (nsecs / ONE_THOUSAND);
  EXPECT_EQ(test_secs, TIME);
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, valid_ros_timestamp) {
  constexpr auto MAX = std::numeric_limits<uint64_t>::max();
  constexpr auto TIME = static_cast<uint64_t>(1554122338652775);
  EXPECT_FALSE(valid_ros_timestamp(MAX));
  EXPECT_TRUE(valid_ros_timestamp(TIME));

  constexpr auto BOUNDARY =
      (static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) *
       ONE_MILLION);
  EXPECT_TRUE(valid_ros_timestamp(BOUNDARY));
  EXPECT_FALSE(valid_ros_timestamp(BOUNDARY + ONE_MILLION));
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, get_unit_enum) {
  EXPECT_EQ(get_unit_enum("null"), Units::null);
  EXPECT_EQ(get_unit_enum("Unit_Bar"), Units::Unit_Bar);
  EXPECT_EQ(get_unit_enum("Unit_PerCent"), Units::Unit_PerCent);
  EXPECT_EQ(get_unit_enum("Unit_DegreOfArc"), Units::Unit_DegreOfArc);
  EXPECT_EQ(get_unit_enum("Unit_KiloMeterPerHour"),
            Units::Unit_KiloMeterPerHour);
  EXPECT_EQ(get_unit_enum("Unit_MeterPerSeconSquar"),
            Units::Unit_MeterPerSeconSquar);
  EXPECT_EQ(get_unit_enum("Unit_DegreOfArcPerSecon"),
            Units::Unit_DegreOfArcPerSecon);
  EXPECT_EQ(get_unit_enum(""), Units::UNKNOWN);
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, to_ros_units) {
  const auto deg_to_rad = [](double deg) { return (deg * (M_PI / 180.0)); };
  const auto kph_to_mps = [](double kph) {
    return (kph * (1000.0 / (60.0 * 60.0)));
  };
  const auto percent_to_unit = [](double percent) { return (percent / 100.0); };

  {
    constexpr auto val = 36.37;
    EXPECT_EQ(to_ros_units("Unit_DegreOfArc", val), deg_to_rad(val));
    EXPECT_EQ(to_ros_units("Unit_DegreOfArcPerSecon", val), deg_to_rad(val));
  }

  {
    constexpr auto val = 15.0;
    EXPECT_EQ(to_ros_units("Unit_KiloMeterPerHour", val), kph_to_mps(val));
  }

  {
    constexpr auto val = 99.1;
    EXPECT_EQ(to_ros_units("Unit_PerCent", val), percent_to_unit(val));
  }

  {
    constexpr auto val = 13.05;
    EXPECT_EQ(to_ros_units("null", val), val);
    EXPECT_EQ(to_ros_units("Unit_Bar", val), val);
    EXPECT_EQ(to_ros_units("Unit_MeterPerSeconSquar", val), val);
  }

  {
    constexpr auto val = 0.0;
    const auto ros_units = to_ros_units("UNKNOWN", val);
    EXPECT_TRUE(std::isnan(ros_units));
  }
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros

