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

#include "a2d2_to_ros/checks.hpp"

static constexpr auto EPS = 1e-8;
static constexpr auto INF = std::numeric_limits<double>::infinity();
static constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();
static constexpr auto ONE_MILLION = static_cast<uint64_t>(1000000);
static constexpr auto ONE_THOUSAND = static_cast<uint64_t>(1000);

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS_checks, strictly_non_negative) {
  EXPECT_TRUE(strictly_non_negative(1));
  EXPECT_TRUE(strictly_non_negative(0));
  EXPECT_FALSE(strictly_non_negative(-1));
  EXPECT_FALSE(strictly_non_negative(NaN));
  EXPECT_FALSE(strictly_non_negative(-INF));
  EXPECT_TRUE(strictly_non_negative(INF));
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS_checks, strictly_positive) {
  EXPECT_TRUE(strictly_positive(1));
  EXPECT_FALSE(strictly_positive(0));
  EXPECT_FALSE(strictly_positive(-1));
  EXPECT_FALSE(strictly_positive(NaN));
  EXPECT_FALSE(strictly_positive(-INF));
  EXPECT_TRUE(strictly_positive(INF));
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS_checks, axis_is_valid) {
  {
    const Eigen::Vector3d axis(1.0, 0.0, INF);
    EXPECT_FALSE(axis_is_valid(axis, EPS));
  }

  {
    const Eigen::Vector3d axis(1.0, 0.0, NaN);
    EXPECT_FALSE(axis_is_valid(axis, EPS));
  }

  {
    const Eigen::Vector3d axis(1.0, 0.0, 0.0);
    EXPECT_TRUE(axis_is_valid(axis, EPS));
  }

  {
    const Eigen::Vector3d axis(0.0, 0.0, 0.0);
    EXPECT_FALSE(axis_is_valid(axis, EPS));
  }

  {
    const Eigen::Vector3d axis(0.0, 0.0, EPS);
    EXPECT_FALSE(axis_is_valid(axis, EPS));
  }

  {
    const Eigen::Vector3d axis(EPS, EPS, EPS);
    EXPECT_TRUE(axis_is_valid(axis, EPS));
  }
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS_checks, axes_are_valid) {
  {
    const Eigen::Vector3d a1(1.0, 0.0, 0.0);
    const Eigen::Vector3d a2(0.0, 1.0, 0.0);
    EXPECT_TRUE(axes_are_valid(a1, a2, EPS));
  }

  {
    const Eigen::Vector3d a1(1.0, 0.0, 0.0);
    const Eigen::Vector3d a2(1.0, 0.0, 0.0);
    EXPECT_FALSE(axes_are_valid(a1, a2, EPS));
  }

  {
    const Eigen::Vector3d a1(1.0, 0.0, 0.0);
    const Eigen::Vector3d a2(1.0 + EPS, 0.0, 0.0);
    EXPECT_FALSE(axes_are_valid(a1, a2, EPS));
  }
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS_checks, verify_ego_bbox_params) {
  {
    const auto x_min = 0.0;
    const auto x_max = 0.0;
    const auto y_min = 0.0;
    const auto y_max = 0.0;
    const auto z_min = 0.0;
    const auto z_max = 0.0;
    const auto zero_measure_valid =
        verify_ego_bbox_params(x_min, x_max, y_min, y_max, z_min, z_max);
    EXPECT_FALSE(zero_measure_valid);
  }

  {
    const auto x_min = 0.0;
    const auto x_max = 1.0;
    const auto y_min = 0.0;
    const auto y_max = 1.0;
    const auto z_min = 0.0;
    const auto z_max = 1.0;
    const auto unit_cube_valid =
        verify_ego_bbox_params(x_min, x_max, y_min, y_max, z_min, z_max);
    EXPECT_TRUE(unit_cube_valid);
  }

  {
    const auto x_min = 0.0;
    const auto x_max = 1.0;
    const auto y_min = 1.0;
    const auto y_max = 0.0;
    const auto z_min = 0.0;
    const auto z_max = 1.0;
    const auto y_wrong_order =
        verify_ego_bbox_params(x_min, x_max, y_min, y_max, z_min, z_max);
    EXPECT_FALSE(y_wrong_order);
  }

  {
    const auto x_min = 0.0;
    const auto x_max = 1.0;
    const auto y_min = 0.0;
    const auto y_max = 1.0;
    const auto z_min = NaN;
    const auto z_max = 1.0;
    const auto non_finite_value =
        verify_ego_bbox_params(x_min, x_max, y_min, y_max, z_min, z_max);
    EXPECT_FALSE(non_finite_value);
  }

  {
    const auto x_min = INF;
    const auto x_max = 1.0;
    const auto y_min = 0.0;
    const auto y_max = 1.0;
    const auto z_min = 0.0;
    const auto z_max = 1.0;
    const auto infinite_value =
        verify_ego_bbox_params(x_min, x_max, y_min, y_max, z_min, z_max);
    EXPECT_FALSE(infinite_value);
  }
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS_checks, valid_ros_timestamp) {
  constexpr auto MAX = std::numeric_limits<uint64_t>::max();
  constexpr auto TIME = static_cast<uint64_t>(1554122338652775);
  EXPECT_FALSE(valid_ros_timestamp(MAX));
  EXPECT_TRUE(valid_ros_timestamp(TIME));

  constexpr auto BOUNDARY =
      (static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) *
       ONE_MILLION);
  EXPECT_TRUE(valid_ros_timestamp(BOUNDARY));
  EXPECT_FALSE(valid_ros_timestamp(BOUNDARY + ONE_MILLION));
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros

