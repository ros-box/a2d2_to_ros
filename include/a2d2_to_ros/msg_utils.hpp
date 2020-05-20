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
#ifndef A2D2_TO_ROS__MSG_UTILS_HPP_
#define A2D2_TO_ROS__MSG_UTILS_HPP_

#include <string>

#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/SolidPrimitive.h>

namespace a2d2_to_ros {

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

}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__MSG_UTILS_HPP_
