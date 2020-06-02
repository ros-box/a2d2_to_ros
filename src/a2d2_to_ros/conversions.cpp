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
#include "a2d2_to_ros/conversions.hpp"

static constexpr auto ONE_THOUSAND = static_cast<uint64_t>(1000);
static constexpr auto ONE_MILLION = static_cast<uint64_t>(1000000);

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

uint64_t TAI_to_UTC(uint64_t mu_s) {
  constexpr auto DELTA_AT_RECORD_TIME =
      (static_cast<uint64_t>(37) * ONE_MILLION);
  return (mu_s - DELTA_AT_RECORD_TIME);
}

//------------------------------------------------------------------------------

size_t flatten_2d_index(size_t width, size_t row, size_t col) {
  return ((row * width) + col);
}

//------------------------------------------------------------------------------

ros::Time a2d2_timestamp_to_ros_time(uint64_t time) {
  const auto truncated_secs = (time / ONE_MILLION);
  const auto mu_secs = (time - (truncated_secs * ONE_MILLION));
  const auto n_secs = (mu_secs * ONE_THOUSAND);
  return ros::Time(static_cast<uint32_t>(truncated_secs),
                   static_cast<uint32_t>(n_secs));
}

//------------------------------------------------------------------------------

Units get_unit_enum(const std::string& unit_name) {
  if (unit_name == "null") {
    return Units::null;
  }
  if (unit_name == "Unit_Bar") {
    return Units::Unit_Bar;
  }
  if (unit_name == "Unit_PerCent") {
    return Units::Unit_PerCent;
  }
  if (unit_name == "Unit_DegreOfArc") {
    return Units::Unit_DegreOfArc;
  }
  if (unit_name == "Unit_KiloMeterPerHour") {
    return Units::Unit_KiloMeterPerHour;
  }
  if (unit_name == "Unit_MeterPerSeconSquar") {
    return Units::Unit_MeterPerSeconSquar;
  }
  if (unit_name == "Unit_DegreOfArcPerSecon") {
    return Units::Unit_DegreOfArcPerSecon;
  }
  return Units::UNKNOWN;
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros
