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
#include "a2d2_to_ros/sensors.hpp"

namespace a2d2_to_ros {
namespace sensors {

//------------------------------------------------------------------------------

const std::string Names::LIDARS = "lidars";
const std::string Names::CAMERAS = "cameras";

//------------------------------------------------------------------------------

std::array<std::string, 8> Frames::get_files() {
  return {"frontcenter", "frontleft",  "frontright", "sideleft",
          "sideright",   "rearcenter", "rearleft",   "rearright"};
}

//------------------------------------------------------------------------------

std::array<std::string, 8> Frames::get_sensors() {
  return {"front_center", "front_left",  "front_right", "side_left",
          "side_right",   "rear_center", "rear_left",   "rear_right"};
}

//------------------------------------------------------------------------------

}  // namespace sensors
}  // namespace a2d2_to_ros
