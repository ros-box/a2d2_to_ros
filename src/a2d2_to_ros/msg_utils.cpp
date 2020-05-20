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
#include "a2d2_to_ros/msg_utils.hpp"

#include <sensor_msgs/point_cloud2_iterator.h>

#include "a2d2_to_ros/npz.hpp"

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

shape_msgs::SolidPrimitive build_ego_shape_msg(double x_min, double x_max,
                                               double y_min, double y_max,
                                               double z_min, double z_max) {
  const auto side_length = [](double min, double max) { return (max - min); };

  auto msg = shape_msgs::SolidPrimitive();
  msg.type = shape_msgs::SolidPrimitive::BOX;
  msg.dimensions.resize(3);
  msg.dimensions[shape_msgs::SolidPrimitive::BOX_X] = side_length(x_min, x_max);
  msg.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = side_length(y_min, y_max);
  msg.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = side_length(z_min, z_max);
  return msg;
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
  const auto float_width = (6 * sizeof(npz::WriteTypes::FLOAT));
  const auto int_width = (2 * sizeof(npz::WriteTypes::UINT64));
  const auto bool_width = (4 * sizeof(npz::WriteTypes::UINT8));
  msg.point_step = (float_width + int_width + bool_width);

  msg.row_step = (3 * msg.point_step);
  msg.is_dense = is_dense;

  const auto fields = npz::Fields::get_fields();
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
      14, "x", 1, npz::WriteTypes::MSG_FLOAT, "y", 1,
      npz::WriteTypes::MSG_FLOAT, "z", 1, npz::WriteTypes::MSG_FLOAT,
      fields[npz::Fields::AZIMUTH_IDX].c_str(), 1, npz::WriteTypes::MSG_FLOAT,
      fields[npz::Fields::BOUNDARY_IDX].c_str(), 1, npz::WriteTypes::MSG_UINT8,
      fields[npz::Fields::COL_IDX].c_str(), 1, npz::WriteTypes::MSG_FLOAT,
      fields[npz::Fields::DEPTH_IDX].c_str(), 1, npz::WriteTypes::MSG_FLOAT,
      fields[npz::Fields::DISTANCE_IDX].c_str(), 1, npz::WriteTypes::MSG_FLOAT,
      fields[npz::Fields::ID_IDX].c_str(), 1, npz::WriteTypes::MSG_UINT8,
      fields[npz::Fields::RECTIME_IDX].c_str(), 1, npz::WriteTypes::MSG_UINT64,
      fields[npz::Fields::ROW_IDX].c_str(), 1, npz::WriteTypes::MSG_FLOAT,
      fields[npz::Fields::REFLECTANCE_IDX].c_str(), 1,
      npz::WriteTypes::MSG_UINT8, fields[npz::Fields::TIMESTAMP_IDX].c_str(), 1,
      npz::WriteTypes::MSG_UINT64, fields[npz::Fields::VALID_IDX].c_str(), 1,
      npz::WriteTypes::MSG_UINT8);

  modifier.resize(msg.width);

  return msg;
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros
