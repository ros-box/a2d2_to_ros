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
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <limits>
#include <utility>

static constexpr auto ONE_THOUSAND = static_cast<uint64_t>(1000);
static constexpr auto ONE_MILLION = static_cast<uint64_t>(1000000);

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

std::array<std::string, 12> get_npz_fields() {
  return {"pcloud_points",           "pcloud_attr.azimuth",
          "pcloud_attr.boundary",    "pcloud_attr.col",
          "pcloud_attr.depth",       "pcloud_attr.distance",
          "pcloud_attr.lidar_id",    "pcloud_attr.rectime",
          "pcloud_attr.reflectance", "pcloud_attr.row",
          "pcloud_attr.timestamp",   "pcloud_attr.valid"};
}

//------------------------------------------------------------------------------

bool verify_npz_structure(const std::map<std::string, cnpy::NpyArray>& npz) {
  ///
  /// Make sure all required fields are there
  ///

  const auto fields = get_npz_fields();
  if (npz.size() != fields.size()) {
    X_ERROR("Expected npz to have " << fields.size() << " fields, but it has "
                                    << npz.size());
    return false;
  }

  for (const auto& f : fields) {
    if (npz.find(f) == std::end(npz)) {
      X_ERROR("Expected npz to have field '" << f << "', but it does not.");
      return false;
    }
  }

  ///
  /// Make sure all fields have expected shape
  ///

  const auto points_field_name = fields[lidar::POINTS_IDX];

  // this cannot throw if the fields check passes
  const auto& points_shape = npz.at(points_field_name).shape;

  if (points_shape.size() != 2) {
    X_ERROR("Points array must have exactly two dimensions. Instead it has "
            << points_shape.size());
    return false;
  }

  if (points_shape[lidar::COL_SHAPE_IDX] != 3) {
    X_ERROR(
        "Points in the points array must have three dimensions. Instead they "
        "have "
        << points_shape[COL_DIM]);
    return false;
  }

  for (const auto& p : npz) {
    const auto& field = p.first;
    // this one is already checked
    if (field == points_field_name) {
      continue;
    }

    // this cannot throw if the fields check passes
    const auto& shape = npz.at(field).shape;

    if (shape.size() != 1) {
      X_ERROR(
          "Expected " << field
                      << " data to have exactly one dimension. Instead it has "
                      << shape.size());
      return false;
    }

    if (shape[lidar::ROW_SHAPE_IDX] != points_shape[lidar::ROW_SHAPE_IDX]) {
      X_ERROR("Expected " << field << " to have exactly " << points_shape[0]
                          << " rows. Instead it has " << shape[ROW_DIM]);
      return false;
    }
  }

  return true;
}

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

size_t flatten_2d_index(size_t width, size_t row, size_t col) {
  return ((row * width) + col);
}

//------------------------------------------------------------------------------

bool valid_ros_timestamp(uint64_t time) {
  const auto secs = (time / ONE_MILLION);
  const auto boundary =
      static_cast<uint64_t>(std::numeric_limits<uint32_t>::max());
  return (secs <= boundary);
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

