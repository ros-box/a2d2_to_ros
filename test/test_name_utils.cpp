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
#include <gtest/gtest.h>

#include "a2d2_to_ros/name_utils.hpp"

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS_name_utils, camera_name_from_lidar_name) {
  {
    const auto name = std::string("20190401145936_lidar_frontcenter_000000080");
    const auto camera_name = camera_name_from_lidar_name(name);
    const auto camera_name_expected =
        std::string("20190401145936_camera_frontcenter_000000080");
    EXPECT_EQ(camera_name, camera_name_expected);
  }

  {
    const auto name = std::string("lidar");
    const auto camera_name = camera_name_from_lidar_name(name);
    const auto camera_name_expected = std::string("camera");
    EXPECT_EQ(camera_name, camera_name_expected);
  }

  {
    const auto name = std::string("20190401145936_ldar_frontcenter_000000080");
    const auto camera_name = camera_name_from_lidar_name(name);
    const auto camera_name_expected = std::string("");
    EXPECT_EQ(camera_name, camera_name_expected);
  }
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros

