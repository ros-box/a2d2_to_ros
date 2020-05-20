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
#include "a2d2_to_ros/npz.hpp"

#include "a2d2_to_ros/checks.hpp"
#include "a2d2_to_ros/logging.hpp"

namespace a2d2_to_ros {
namespace npz {

//------------------------------------------------------------------------------

std::array<std::string, 12> Fields::get_fields() {
  return {"pcloud_points",           "pcloud_attr.azimuth",
          "pcloud_attr.boundary",    "pcloud_attr.col",
          "pcloud_attr.depth",       "pcloud_attr.distance",
          "pcloud_attr.lidar_id",    "pcloud_attr.rectime",
          "pcloud_attr.reflectance", "pcloud_attr.row",
          "pcloud_attr.timestamp",   "pcloud_attr.valid"};
}

//------------------------------------------------------------------------------

bool verify_structure(const std::map<std::string, cnpy::NpyArray>& npz) {
  ///
  /// Make sure all required fields are there
  ///

  const auto fields = Fields::get_fields();
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

  const auto points_field_name = fields[Fields::POINTS_IDX];

  // this cannot throw if the fields check passes
  const auto& points_shape = npz.at(points_field_name).shape;

  if (points_shape.size() != 2) {
    X_ERROR("Points array must have exactly two dimensions. Instead it has "
            << points_shape.size());
    return false;
  }

  if (points_shape[Fields::COL_SHAPE_IDX] != 3) {
    X_ERROR(
        "Points in the points array must have three dimensions. Instead they "
        "have "
        << points_shape[Fields::COL_SHAPE_IDX]);
    return false;
  }

  for (const auto& p : npz) {
    const auto& field_name = p.first;
    const auto& field_values = p.second;
    // this one is already checked
    if (field_name == points_field_name) {
      continue;
    }

    const auto& shape = field_values.shape;

    if (shape.size() != 1) {
      X_ERROR(
          "Expected " << field_name
                      << " data to have exactly one dimension. Instead it has "
                      << shape.size());
      return false;
    }

    if (shape[Fields::ROW_SHAPE_IDX] != points_shape[Fields::ROW_SHAPE_IDX]) {
      X_ERROR("Expected " << field_name << " to have exactly "
                          << points_shape[0] << " rows. Instead it has "
                          << shape[Fields::ROW_SHAPE_IDX]);
      return false;
    }

    ///
    /// Make sure fields have expected sign
    ///

    auto sign_error = false;

    const auto is_timestamp = (field_name == fields[Fields::TIMESTAMP_IDX]);
    const auto is_rectime = (field_name == fields[Fields::RECTIME_IDX]);
    const auto is_lidar_id = (field_name == fields[Fields::ID_IDX]);
    if (is_timestamp || is_rectime || is_lidar_id) {
      const auto non_negative = all_non_negative<int64_t>(field_values);
      sign_error = (sign_error || !non_negative);
    }

    const auto is_row = (field_name == fields[Fields::ROW_IDX]);
    const auto is_col = (field_name == fields[Fields::COL_IDX]);
    const auto is_depth = (field_name == fields[Fields::DEPTH_IDX]);
    const auto is_distance = (field_name == fields[Fields::DISTANCE_IDX]);
    // TODO(jeff): figure out whether row/col can be negative
    if (/* is_row || is_col ||*/ is_depth || is_distance) {
      const auto non_negative =
          all_non_negative<ReadTypes::Point>(field_values);
      sign_error = (sign_error || !non_negative);
    }

    if (sign_error) {
      X_ERROR("Expected " << field_name
                          << " to be strictly non-negative. Instead, it has "
                             "negative values.");
      return false;
    }

    ///
    /// Make sure times are compatible with ROS
    /// TODO(jeff): Add rectime here once it's verified that that's a timestamp
    ///
    if (is_timestamp) {
      const auto length = field_values.shape[Fields::ROW_SHAPE_IDX];
      const auto& data = field_values.data<ReadTypes::Timestamp>();
      for (auto i = 0; i < length; ++i) {
        // preceding checks guarantee data is non-negative
        const auto t = static_cast<uint64_t>(data[i]);
        if (!valid_ros_timestamp(t)) {
          X_ERROR("Timestamp "
                  << t
                  << " has unsupported magnitude: ROS does not support "
                     "timestamps on or after 4294967296000000 "
                     "(Sunday, February 7, 2106 6:28:16 AM GMT)\nCall "
                     "Zager and Evans for details.");
          return false;
        }
      }
    }
  }

  return true;
}

//------------------------------------------------------------------------------

bool any_points_invalid(const cnpy::NpyArray& valid) {
  auto all_valid = true;
  const auto v = valid.data<bool>();
  for (auto i = 0; i < valid.shape[Fields::ROW_SHAPE_IDX]; ++i) {
    all_valid = (all_valid && v[i]);
  }
  return !all_valid;
}

//------------------------------------------------------------------------------

}  // namespace npz
}  // namespace a2d2_to_ros
