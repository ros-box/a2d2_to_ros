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
#include "a2d2_to_ros/point_cloud_iterators.hpp"

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

A2D2_PointCloudIterators::A2D2_PointCloudIterators(
    sensor_msgs::PointCloud2& msg, const std::array<std::string, 12>& fields)
    : x(msg, "x"),
      y(msg, "y"),
      z(msg, "z"),
      azimuth(msg, fields[npz::Fields::AZIMUTH_IDX]),
      boundary(msg, fields[npz::Fields::BOUNDARY_IDX]),
      col(msg, fields[npz::Fields::COL_IDX]),
      depth(msg, fields[npz::Fields::DEPTH_IDX]),
      distance(msg, fields[npz::Fields::DISTANCE_IDX]),
      lidar_id(msg, fields[npz::Fields::ID_IDX]),
      rectime(msg, fields[npz::Fields::RECTIME_IDX]),
      reflectance(msg, fields[npz::Fields::REFLECTANCE_IDX]),
      row(msg, fields[npz::Fields::ROW_IDX]),
      timestamp(msg, fields[npz::Fields::TIMESTAMP_IDX]),
      valid(msg, fields[npz::Fields::VALID_IDX]) {}

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
  os << "{"
     << "x: " << *(iters.x) << ", y: " << *(iters.y) << ", z: " << *(iters.z)
     << ", azimuth: " << *(iters.azimuth) << ", boundary: " << *(iters.boundary)
     << ", col: " << *(iters.col) << ", depth: " << *(iters.depth)
     << ", distance: " << *(iters.distance)
     << ", lidar_id: " << static_cast<int>(*(iters.lidar_id))
     << ", rectime: " << *(iters.rectime)
     << ", reflectance: " << static_cast<int>(*(iters.reflectance))
     << ", row: " << *(iters.row) << ", timestamp: " << *(iters.timestamp)
     << ", valid: " << *(iters.valid) << "}";
  return os;
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros
