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

#include "a2d2_to_ros/file_utils.hpp"

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS_file_utils, frame_from_filename) {
  {
    const auto frame_expected = "frontcenter";
    const auto filename =
        "/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/lidar/"
        "cam_front_center/20190401145936_lidar_frontcenter_000000080.npz";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "frontcenter";
    const auto filename = "20190401145936_lidar_frontcenter_000000080.npz";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "frontcenter";
    const auto filename = "20190401145936_lidar_frontcenter_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "";
    const auto filename = "20190401145936_lidar_frontcenter_sideleft_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "";
    const auto filename = "20190401145936_lidar_front_center_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "frontleft";
    const auto filename = "20190401145936_lidar_frontleft_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "frontright";
    const auto filename = "20190401145936_lidar_frontright_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "sideleft";
    const auto filename = "20190401145936_lidar_sideleft_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "sideright";
    const auto filename = "20190401145936_lidar_sideright_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "rearcenter";
    const auto filename = "20190401145936_lidar_rearcenter_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "rearleft";
    const auto filename = "20190401145936_lidar_rearleft_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "rearright";
    const auto filename = "20190401145936_lidar_rearright_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "";
    const auto filename = "20190401145936_lidar_rear_left_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "";
    const auto filename = "20190401145936_lidar_rear_right_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros

