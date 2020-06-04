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
#ifndef A2D2_TO_ROS__NAME_UTILS_HPP_
#define A2D2_TO_ROS__NAME_UTILS_HPP_

#include <string>

namespace a2d2_to_ros {

/**
 * @brief Convenience method to generate a standard TF frame name.
 * @note This function has no test coverage.
 */
std::string tf_motion_compensated_sensor_frame_name(
    const std::string& sensor_type, const std::string& sensor_frame);

/**
 * @brief Convenience method to generate a standard TF frame name.
 * @note This function has no test coverage.
 */
std::string tf_frame_name(const std::string& sensor_type,
                          const std::string& sensor_frame);

/**
 * @brief Get camera file basename corresponding to the given lidar basename.
 * @pre The input must be a basename (no directory and no extension)
 * @return The basename with 'lidar' replaced by 'camera', or the empty string
 * if 'lidar' is not present in the input.
 */
std::string camera_name_from_lidar_name(const std::string& basename);

/**
 * @brief Map camera name from npz lidar filename to camera name
 * @note This function has no test coverage.
 * @return The camera name corresponding to the lidar frame, or empty string if
 * not found.
 */
std::string get_camera_name_from_frame_name(const std::string& name);

}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__NAME_UTILS_HPP_
