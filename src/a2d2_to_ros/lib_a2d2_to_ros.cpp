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
#define _ENABLE_A2D2_ROS_LOGGING_
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"

#include <cstdint>
#include <fstream>
#include <limits>
#include <utility>

static constexpr auto ONE_THOUSAND = static_cast<uint64_t>(1000);
static constexpr auto ONE_MILLION = static_cast<uint64_t>(1000000);

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

DataPair DataPair::build(double value, uint64_t time, std::string frame_id) {
  std_msgs::Header header;
  header.seq = 0;
  header.frame_id = std::move(frame_id);
  header.stamp = a2d2_timestamp_to_ros_time(time);

  std_msgs::Float64 val;
  val.data = value;

  return DataPair(header, val);
}

//------------------------------------------------------------------------------

DataPair::DataPair(std_msgs::Header header, std_msgs::Float64 value)
    : header(std::move(header)), value(std::move(value)) {}

//------------------------------------------------------------------------------

bool DataPairTimeComparator::operator()(const DataPair& lhs,
                                        const DataPair& rhs) const {
  return (lhs.header.stamp < rhs.header.stamp);
}

//------------------------------------------------------------------------------

bool valid_a2d2_timestamp(uint64_t time) {
  const auto secs = (time / ONE_MILLION);
  return (secs <= std::numeric_limits<uint32_t>::max());
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

std::string get_json_file_as_string(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs.good()) {
    return std::string("");
  }

  std::string content((std::istreambuf_iterator<char>(ifs)),
                      (std::istreambuf_iterator<char>()));
  return content;
}

//------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, Units u) {
  switch (u) {
    case Units::null:
      os << "null";
      break;
    case Units::Unit_Bar:
      os << "Unit_Bar";
      break;
    case Units::Unit_PerCent:
      os << "Unit_PerCent";
      break;
    case Units::Unit_DegreOfArc:
      os << "Unit_DegreOfArc";
      break;
    case Units::Unit_KiloMeterPerHour:
      os << "Unit_KiloMeterPerHour";
      break;
    case Units::Unit_MeterPerSeconSquar:
      os << "Unit_MeterPerSeconSquar";
      break;
    case Units::Unit_DegreOfArcPerSecon:
      os << "Unit_DegreOfArcPerSecon";
      break;
    default:
      os << "UNKNOWN";
      break;
  }
  return os;
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

  X_ERROR("Unrecogized units name: '" << unit_name
                                      << "'. Returning 'UNKNOWN'.");
  return Units::UNKNOWN;
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros

