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

#include "a2d2_to_ros/transform_utils.hpp"

static constexpr auto EPS = 1e-8;

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS_transform_utils, get_orthonormal_basis) {
  {
    const Eigen::Vector3d X(1.0, 0.0, 0.0);
    const Eigen::Vector3d Y(0.0, 1.0, 0.0);
    const Eigen::Matrix3d B = get_orthonormal_basis(X, Y, EPS);
    Eigen::Matrix3d B_expected;
    // clang-format off
    B_expected <<
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0;
    // clang-format on
    EXPECT_TRUE(B.transpose().isApprox(B_expected, EPS));
  }

  {
    const Eigen::Vector3d X(1.0, 1.0, 1.0);
    const Eigen::Vector3d Y(0.0, 1.0, 0.0);
    const Eigen::Matrix3d B = get_orthonormal_basis(X, Y, EPS);
    Eigen::Matrix3d B_expected;
    // clang-format off
    B_expected <<
       0.57735026918962584,  0.57735026918962584,  0.57735026918962584,
      -0.40824829046386307,  0.81649658092772615, -0.40824829046386307,
      -0.70710678118654746,                    0,  0.70710678118654746;
    // clang-format on
    EXPECT_TRUE(B.transpose().isApprox(B_expected, EPS))
        << "B: " << B << "\nDid not match B_expected: " << B_expected;
  }
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros

