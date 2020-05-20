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
#ifndef A2D2_TO_ROS__POINT_CLOUD_ITERATORS_HPP_
#define A2D2_TO_ROS__POINT_CLOUD_ITERATORS_HPP_

#include <array>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "a2d2_to_ros/npz.hpp"

namespace a2d2_to_ros {

/** @brief convenience object for interacting with point cloud iterators. */
struct A2D2_PointCloudIterators {
  A2D2_PointCloudIterators(sensor_msgs::PointCloud2& msg,
                           const std::array<std::string, 12>& fields);

  /** @brief Convenience overload to in-place pre-increment all iterators. */
  void operator++();

  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Point> x;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Point> y;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Point> z;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Azimuth> azimuth;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Boundary> boundary;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Col> col;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Depth> depth;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Distance> distance;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::LidarId> lidar_id;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Rectime> rectime;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Reflectance> reflectance;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Row> row;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Timestamp> timestamp;
  sensor_msgs::PointCloud2Iterator<npz::WriteTypes::Valid> valid;
};  // struct A2D2_PointCloudIterators

/**
 * @brief Print all current values to stream.
 * @pre All iterators point to valid values.
 */
std::ostream& operator<<(std::ostream& os,
                         const A2D2_PointCloudIterators& iters);

}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__POINT_CLOUD_ITERATORS_HPP_
