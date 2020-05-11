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

#define _USE_MATH_DEFINES
#include <cmath>

#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include "ros_cnpy/cnpy.h"

#include "a2d2_to_ros/logging.hpp"

namespace a2d2_to_ros {
namespace lidar {
constexpr auto POINTS_IDX = 0;
constexpr auto AZIMUTH_IDX = 1;
constexpr auto BOUNDARY_IDX = 2;
constexpr auto COL_IDX = 3;
constexpr auto DEPTH_IDX = 4;
constexpr auto DISTANCE_IDX = 5;
constexpr auto ID_IDX = 6;
constexpr auto RECTIME_IDX = 7;
constexpr auto REFLECTANCE_IDX = 8;
constexpr auto ROW_IDX = 9;
constexpr auto TIMESTAMP_IDX = 10;
constexpr auto VALID_DIX = 11;

constexpr auto ROW_SHAPE_IDX = 0;
constexpr auto COL_SHAPE_IDX = 1;

/**
 * @brief Explicit notion of data types: these types are used to read data from
 * the numpy npz files.
 */
struct ReadTypes {
  typedef double FLOAT;
  typedef int64_t INT64;
  typedef bool BOOL;

  typedef FLOAT Point;
  typedef FLOAT Azimuth;
  typedef FLOAT Col;
  typedef FLOAT Depth;
  typedef FLOAT Distance;
  typedef FLOAT Row;

  typedef INT64 Boundary;
  typedef INT64 LidarId;
  typedef INT64 Rectime;
  typedef INT64 Reflectance;
  typedef INT64 Timestamp;

  typedef BOOL Valid;
};  // struct ReadTypes

/**
 * @brief Explicit notion of data types: these types are used to write data to
 * point cloud messages.
 */
struct WriteTypes {
#ifdef USE_FLOAT64
  typedef double FLOAT;
  static const uint8_t MSG_FLOAT = sensor_msgs::PointField::FLOAT64;
#else
  typedef float FLOAT;
  static const uint8_t MSG_FLOAT = sensor_msgs::PointField::FLOAT32;
#endif
  static const uint8_t MSG_UINT64 = sensor_msgs::PointField::FLOAT64;
  static const uint8_t MSG_UINT8 = sensor_msgs::PointField::UINT8;
  typedef uint64_t UINT64;
  typedef uint8_t UINT8;
  typedef bool BOOL;

  typedef FLOAT Point;
  typedef FLOAT Azimuth;
  typedef FLOAT Col;
  typedef FLOAT Depth;
  typedef FLOAT Distance;
  typedef FLOAT Row;

  typedef UINT64 Rectime;
  typedef UINT64 Timestamp;

  typedef UINT8 Reflectance;
  typedef UINT8 LidarId;

  typedef BOOL Boundary;
  typedef BOOL Valid;
};  // struct ReadTypes

}  // namespace lidar

/** @brief convenience object for interacting with point cloud iterators. */
struct A2D2_PointCloudIterators {
  A2D2_PointCloudIterators(sensor_msgs::PointCloud2& msg,
                           const std::array<std::string, 12>& fields);

  /** @brief Convenience overload to in-place pre-increment all iterators. */
  void operator++();

  /**
   * @brief Print all current values to stream.
   * @pre All iterators point to valid values.
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const A2D2_PointCloudIterators& iters);

  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Point> x;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Point> y;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Point> z;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Azimuth> azimuth;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Boundary> boundary;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Col> col;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Depth> depth;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Distance> distance;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::LidarId> lidar_id;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Rectime> rectime;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Reflectance> reflectance;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Row> row;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Timestamp> timestamp;
  sensor_msgs::PointCloud2Iterator<lidar::WriteTypes::Valid> valid;
};  // struct A2D2_PointCloudIterators

sensor_msgs::ImagePtr depth_image_from_a2d2_pointcloud(
    sensor_msgs::PointCloud2& pc);

/**
 * @brief Check whether the ego bbox parameters makes sense.
 * @return true iff the values are valid, i.e., real valued, finite, ordered,
 * and the bbox has non-zero measure.
 */
bool verify_ego_bbox_params(double x_min, double x_max, double y_min,
                            double y_max, double z_min, double z_max);

/**
 * @brief Build a ROS message corresponding to the vehicle bbox extents.
 * @pre The values are correct according to 'verify_ego_bbox_params'
 * @return The shape message representing the ego bounding box. By convention
 * the side lengths given are end-to-end side lengths.
 */
shape_msgs::SolidPrimitive build_ego_shape_msg(double x_min, double x_max,
                                               double y_min, double y_max,
                                               double z_min, double z_max);

/**
 * @brief Build a PointCloud2 message for storing points from a single npz file.
 * @return A property configured and sized, but uninitialized, PointCloud2
 * message object.
 */
sensor_msgs::PointCloud2 build_pc2_msg(std::string frame, ros::Time timestamp,
                                       bool is_dense,
                                       const uint32_t num_points);

/**
 * @brief Get camera file basename corresponding to the given lidar basename.
 * @pre The input must be a basename (no directory and no extension)
 * @return The basename with 'lidar' replaced by 'camera', or the empty string
 * if 'lidar' is not present in the input.
 */
std::string camera_name_from_lidar_name(const std::string& basename);

/**
 * @brief Get the frame of the data from its filename.
 * @return The empty string if a frame name is not present or if multiple
 * different names are present.
 */
std::string frame_from_filename(const std::string& filename);

/**
 * @brief Test whether int64_t data is all non-negative.
 * @note This function has no test coverage.
 * @pre template parameter T must match the underlying data type.
 */
template <typename T>
bool all_non_negative(const cnpy::NpyArray& field) {
  auto good = true;
  const auto vals = field.data<T>();
  for (auto i = 0; i < field.shape[lidar::ROW_SHAPE_IDX]; ++i) {
    const auto is_non_negative = (vals[i] >= static_cast<T>(0));
    good = (good && is_non_negative);
  }
  return good;
}

/**
 * @brief Get the minimum value of a given field
 * @note This function has no test coverage.
 * @pre template parameter T must match the underlying data type.
 * @return The minimum value or std::numeric_limits<T>::max() if field is empty.
 */
template <typename T>
T get_min_value(const cnpy::NpyArray& field) {
  auto t = std::numeric_limits<T>::max();
  const auto vals = field.data<T>();
  for (auto i = 0; i < field.shape[lidar::ROW_SHAPE_IDX]; ++i) {
    t = std::min(t, vals[i]);
  }
  return t;
}

/**
 * @brief Get the maximum value of a given field
 * @note This function has no test coverage.
 * @pre template parameter T must match the underlying data type.
 * @return The maximum value or std::numeric_limits<T>::min() if field is empty.
 */
template <typename T>
T get_max_value(const cnpy::NpyArray& field) {
  auto t = std::numeric_limits<T>::min();
  const auto vals = field.data<T>();
  for (auto i = 0; i < field.shape[lidar::ROW_SHAPE_IDX]; ++i) {
    t = std::max(t, vals[i]);
  }
  return t;
}

/**
 * @brief Check whether the valid array has any false values.
 * @note This function has no test coverage.
 * @pre The underlying type is bool.
 */
bool any_lidar_points_invalid(const cnpy::NpyArray& valid);

/**
 * @brief Get a list of sensor frame names.
 * @note This function has no test coverage.
 */
std::array<std::string, 6> get_sensor_frame_names();

/**
 * @brief Get a list of camera names.
 * @note This function has no test coverage.
 */
std::array<std::string, 6> get_camera_names();

/**
 * @brief Map camera name from npz lidar filename to camera name
 * @note This function has no test coverage.
 * @return The camera name corresponding to the lidar frame, or empty string if
 * not found.
 */
std::string get_camera_name_from_frame_name(const std::string& name);

/**
 * @brief Get a list of expected field names for npz lidar data.
 * @note This function has no test coverage.
 */
std::array<std::string, 12> get_npz_fields();

/**
 * @brief Check that lidar npz data has expected structure.
 * @note This function has no test coverage.
 * @return true iff the npz has exactly the fields from get_npz_fields, and if
 * the point field is M x 3 in size and all other fields are M x 1 in size,
 * where 'M' is defined as the row count of the points field.
 */
bool verify_npz_structure(const std::map<std::string, cnpy::NpyArray>& npz);

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
 * @brief Convert a 2D index to a 1D index according to CNPY convention.
 */
size_t flatten_2d_index(size_t width, size_t row, size_t col);

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

#endif  // A2D2_TO_ROS__LIB_A2D2_TO_ROS_HPP_
