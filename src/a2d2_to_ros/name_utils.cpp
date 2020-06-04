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
#include "a2d2_to_ros/name_utils.hpp"

#include "a2d2_to_ros/sensors.hpp"

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

std::string tf_motion_compensated_sensor_frame_name(
    const std::string& sensor_type, const std::string& sensor_frame) {
  return tf_frame_name(sensor_type, sensor_frame + "_motion_compensated");
}

//------------------------------------------------------------------------------

std::string tf_frame_name(const std::string& sensor_type,
                          const std::string& sensor_frame) {
  return (sensor_type + "_" + sensor_frame);
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

std::string get_camera_name_from_frame_name(const std::string& name) {
  const auto cameras = sensors::Frames::get_sensors();
  const auto frames = sensors::Frames::get_files();
  static_assert((cameras.size() == frames.size()),
                "Sensor names and file names must be same size.");
  for (auto i = 0; i < frames.size(); ++i) {
    if (name == frames[i]) {
      return cameras[i];
    }
  }
  return "";
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros
