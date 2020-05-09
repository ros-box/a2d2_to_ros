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

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <limits>
#include <utility>

static constexpr auto ONE_THOUSAND = static_cast<uint64_t>(1000);
static constexpr auto ONE_MILLION = static_cast<uint64_t>(1000000);

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

A2D2_PointCloudIterators::A2D2_PointCloudIterators(
    sensor_msgs::PointCloud2& msg, const std::array<std::string, 12>& fields)
    : x(msg, "x"),
      y(msg, "y"),
      z(msg, "z"),
      azimuth(msg, fields[lidar::AZIMUTH_IDX]),
      boundary(msg, fields[lidar::BOUNDARY_IDX]),
      col(msg, fields[lidar::COL_IDX]),
      depth(msg, fields[lidar::DEPTH_IDX]),
      distance(msg, fields[lidar::DISTANCE_IDX]),
      lidar_id(msg, fields[lidar::ID_IDX]),
      rectime(msg, fields[lidar::RECTIME_IDX]),
      reflectance(msg, fields[lidar::REFLECTANCE_IDX]),
      row(msg, fields[lidar::ROW_IDX]),
      timestamp(msg, fields[lidar::TIMESTAMP_IDX]),
      valid(msg, fields[lidar::VALID_DIX]) {}

//------------------------------------------------------------------------------

void A2D2_PointCloudIterators::operator++() {
  ++x;
  ++y;
  ++z;
  ++azimuth;
  ++boundary;
  ++col;
  ++depth;
  ++distance;
  ++lidar_id;
  ++rectime;
  ++reflectance;
  ++row;
  ++timestamp;
  ++valid;
}

//------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os,
                         const A2D2_PointCloudIterators& iters) {
  os << "{";
  os << "x: " << *(iters.x) << ", y: " << *(iters.y) << ", z: " << *(iters.z)
     << ", azimuth: " << *(iters.azimuth)
     << ", boundary: " << static_cast<bool>(*(iters.boundary))
     << ", col: " << *(iters.col) << ", depth: " << *(iters.depth)
     << ", distance: " << *(iters.distance)
     << ", lidar_id: " << static_cast<int>(*(iters.lidar_id))
     << ", rectime: " << *(iters.rectime)
     << ", reflectance: " << static_cast<int>(*(iters.reflectance))
     << ", row: " << *(iters.row) << ", timestamp: " << *(iters.timestamp)
     << ", valid: " << static_cast<bool>(*(iters.valid));
  os << "}";
  return os;
}

//------------------------------------------------------------------------------

sensor_msgs::ImagePtr depth_image_from_a2d2_pointcloud(
    sensor_msgs::PointCloud2& pc) {
  const auto fields = get_npz_fields();
  auto iters = A2D2_PointCloudIterators(pc, fields);
  // TODO(jeff):
  // 1. schema for sensor config json
  // 2. parse sensor config json
  // 3. pass in resolution
  // 4. convert to message
  // 5. done
  return sensor_msgs::ImagePtr();
}

//------------------------------------------------------------------------------

sensor_msgs::PointCloud2 build_pc2_msg(std::string frame, ros::Time timestamp,
                                       bool is_dense,
                                       const uint32_t num_points) {
  sensor_msgs::PointCloud2 msg;
  msg.header.seq = static_cast<uint32_t>(0);
  msg.header.stamp = timestamp;
  msg.header.frame_id = std::move(frame);
  msg.height = static_cast<uint32_t>(1);
  msg.width = num_points;

  // cnpy uses little endian format
  msg.is_bigendian = false;

  // use uint8_t for bool; PointField does not define a bool type
  const auto float_width = (6 * sizeof(lidar::WriteTypes::FLOAT));
  const auto int_width = (2 * sizeof(lidar::WriteTypes::INT64));
  const auto bool_width = (4 * sizeof(lidar::WriteTypes::UINT8));
  msg.point_step = (float_width + int_width + bool_width);

  msg.row_step = (3 * msg.point_step);
  msg.is_dense = is_dense;

  const auto fields = get_npz_fields();
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
      14, "x", 1, lidar::WriteTypes::MSG_FLOAT, "y", 1,
      lidar::WriteTypes::MSG_FLOAT, "z", 1, lidar::WriteTypes::MSG_FLOAT,
      fields[a2d2_to_ros::lidar::AZIMUTH_IDX].c_str(), 1,
      lidar::WriteTypes::MSG_FLOAT,
      fields[a2d2_to_ros::lidar::BOUNDARY_IDX].c_str(), 1,
      lidar::WriteTypes::MSG_UINT8, fields[a2d2_to_ros::lidar::COL_IDX].c_str(),
      1, lidar::WriteTypes::MSG_FLOAT,
      fields[a2d2_to_ros::lidar::DEPTH_IDX].c_str(), 1,
      lidar::WriteTypes::MSG_FLOAT,
      fields[a2d2_to_ros::lidar::DISTANCE_IDX].c_str(), 1,
      lidar::WriteTypes::MSG_FLOAT, fields[a2d2_to_ros::lidar::ID_IDX].c_str(),
      1, lidar::WriteTypes::MSG_UINT8,
      fields[a2d2_to_ros::lidar::RECTIME_IDX].c_str(), 1,
      lidar::WriteTypes::MSG_INT64, fields[a2d2_to_ros::lidar::ROW_IDX].c_str(),
      1, lidar::WriteTypes::MSG_FLOAT,
      fields[a2d2_to_ros::lidar::REFLECTANCE_IDX].c_str(), 1,
      lidar::WriteTypes::MSG_UINT8,
      fields[a2d2_to_ros::lidar::TIMESTAMP_IDX].c_str(), 1,
      lidar::WriteTypes::MSG_INT64,
      fields[a2d2_to_ros::lidar::VALID_DIX].c_str(), 1,
      lidar::WriteTypes::MSG_UINT8);

  modifier.resize(msg.width);

  return msg;
}

//------------------------------------------------------------------------------

std::string camera_name_from_lidar_name(const std::string& basename) {
  auto camera_basename = basename;
  const auto lidar = std::string("lidar");
  try {
    const auto pos = basename.find(lidar);
    return camera_basename.replace(pos, lidar.length(), "camera");
  } catch (...) {
    return "";
  }
}

//------------------------------------------------------------------------------

std::string frame_from_filename(const std::string& filename) {
  const auto frames = get_sensor_frame_names();

  const auto filename_has_frame = [&filename](const std::string& frame) {
    return (filename.find(frame) != std::string::npos);
  };

  const auto num_found =
      std::count_if(std::begin(frames), std::end(frames), filename_has_frame);
  if (num_found != 1) {
    return "";
  }

  const auto it =
      std::find_if(std::begin(frames), std::end(frames), filename_has_frame);

  const auto idx = (it - std::begin(frames));
  return frames[idx];
}

//------------------------------------------------------------------------------

bool any_lidar_points_invalid(const cnpy::NpyArray& valid) {
  auto all_valid = true;
  const auto v = valid.data<bool>();
  for (auto i = 0; i < valid.shape[lidar::ROW_SHAPE_IDX]; ++i) {
    all_valid = (all_valid && v[i]);
  }
  return !all_valid;
}

//------------------------------------------------------------------------------

std::array<std::string, 6> get_sensor_frame_names() {
  return {"frontcenter", "frontleft", "frontright",
          "sideleft",    "sideright", "rearcenter"};
}

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
        << points_shape[lidar::COL_SHAPE_IDX]);
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

    if (shape[lidar::ROW_SHAPE_IDX] != points_shape[lidar::ROW_SHAPE_IDX]) {
      X_ERROR("Expected " << field_name << " to have exactly "
                          << points_shape[0] << " rows. Instead it has "
                          << shape[lidar::ROW_SHAPE_IDX]);
      return false;
    }

    ///
    /// Make sure fields have expected sign
    ///

    auto sign_error = false;

    const auto is_timestamp = (field_name == fields[lidar::TIMESTAMP_IDX]);
    const auto is_rectime = (field_name == fields[lidar::RECTIME_IDX]);
    const auto is_lidar_id = (field_name == fields[lidar::ID_IDX]);
    if (is_timestamp || is_rectime || is_lidar_id) {
      const auto non_negative = all_non_negative<int64_t>(field_values);
      sign_error = (sign_error || !non_negative);
    }

    const auto is_row = (field_name == fields[lidar::ROW_IDX]);
    const auto is_col = (field_name == fields[lidar::COL_IDX]);
    const auto is_depth = (field_name == fields[lidar::DEPTH_IDX]);
    const auto is_distance = (field_name == fields[lidar::DISTANCE_IDX]);
    // TODO(jeff): figure out whether row/col can be negative
    if (/* is_row || is_col ||*/ is_depth || is_distance) {
      const auto non_negative =
          all_non_negative<lidar::ReadTypes::Point>(field_values);
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
      const auto length = field_values.shape[lidar::ROW_SHAPE_IDX];
      const auto& data = field_values.data<lidar::ReadTypes::Timestamp>();
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
}  // namespace a2d2_to_ros

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

