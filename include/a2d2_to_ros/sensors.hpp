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
#ifndef A2D2_TO_ROS__SENSORS_HPP_
#define A2D2_TO_ROS__SENSORS_HPP_

#include <array>
#include <string>

namespace a2d2_to_ros {
namespace sensors {

struct Names {
  static const std::string LIDARS;
  static const std::string CAMERAS;
};  // struct Names

struct Frames {
  static constexpr auto FRONT_CENTER_IDX = 0;
  static constexpr auto FRONT_LEFT_IDX = 1;
  static constexpr auto FRONT_RIGHT_IDX = 2;
  static constexpr auto SIDE_LEFT_IDX = 3;
  static constexpr auto SIDE_RIGHT_IDX = 4;
  static constexpr auto REAR_CENTER_IDX = 5;
  static constexpr auto REAR_LEFT_IDX = 6;
  static constexpr auto REAR_RIGHT_IDX = 7;

  /**
   * @brief Get a list of sensor frame names.
   * @note This function has no test coverage.
   * @note I don't know why the data set has two naming conventions (see
   * get_camera_names)
   * @note The names 'rearleft' and 'rearright' are included for completeness
   * but appear to be unused in the data set.
   */
  static std::array<std::string, 8> get_files();

  /**
   * @brief Get a list of sensor names.
   * @note Cameras and lidars use different subsets of these names
   * @note This function has no test coverage.
   * @note I don't know why the data set has two naming conventions (see
   * get_sensor_frame_names)
   */
  static std::array<std::string, 8> get_sensors();
};  // struct Fields

}  // namespace sensors
}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__SENSORS_HPP_
