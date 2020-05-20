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
#ifndef A2D2_TO_ROS__NPZ_HPP_
#define A2D2_TO_ROS__NPZ_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "ros_cnpy/cnpy.h"

#include <sensor_msgs/PointField.h>

namespace a2d2_to_ros {
namespace npz {

struct Fields {
  static constexpr auto POINTS_IDX = 0;
  static constexpr auto AZIMUTH_IDX = 1;
  static constexpr auto BOUNDARY_IDX = 2;
  static constexpr auto COL_IDX = 3;
  static constexpr auto DEPTH_IDX = 4;
  static constexpr auto DISTANCE_IDX = 5;
  static constexpr auto ID_IDX = 6;
  static constexpr auto RECTIME_IDX = 7;
  static constexpr auto REFLECTANCE_IDX = 8;
  static constexpr auto ROW_IDX = 9;
  static constexpr auto TIMESTAMP_IDX = 10;
  static constexpr auto VALID_IDX = 11;

  static constexpr auto ROW_SHAPE_IDX = 0;
  static constexpr auto COL_SHAPE_IDX = 1;

  /**
   * @brief Get a list of expected field names for npz lidar data.
   * @note This function has no test coverage.
   */
  static std::array<std::string, 12> get_fields();
};  // struct Fields

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

/**
 * @brief Check that lidar npz data has expected structure.
 * @note This function has no test coverage.
 * @return true iff the npz has exactly the fields from get_npz_fields, and if
 * the point field is M x 3 in size and all other fields are M x 1 in size,
 * where 'M' is defined as the row count of the points field.
 */
bool verify_structure(const std::map<std::string, cnpy::NpyArray>& npz);

/**
 * @brief Check whether the valid array has any false values.
 * @note This function has no test coverage.
 * @pre The underlying type is bool.
 */
bool any_points_invalid(const cnpy::NpyArray& valid);

/**
 * @brief Test whether int64_t data is all non-negative.
 * @note This function has no test coverage.
 * @pre template parameter T must match the underlying data type.
 */
template <typename T>
bool all_non_negative(const cnpy::NpyArray& field) {
  auto good = true;
  const auto vals = field.data<T>();
  for (auto i = 0; i < field.shape[npz::Fields::ROW_SHAPE_IDX]; ++i) {
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
  for (auto i = 0; i < field.shape[npz::Fields::ROW_SHAPE_IDX]; ++i) {
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
  for (auto i = 0; i < field.shape[npz::Fields::ROW_SHAPE_IDX]; ++i) {
    t = std::max(t, vals[i]);
  }
  return t;
}

}  // namespace npz
}  // namespace a2d2_to_ros

#endif
