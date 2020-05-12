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
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"

static constexpr auto EPS = 1e-8;
static constexpr auto INF = std::numeric_limits<double>::infinity();
static constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();
static constexpr auto ONE_MILLION = static_cast<uint64_t>(1000000);
static constexpr auto ONE_THOUSAND = static_cast<uint64_t>(1000);

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, axis_is_valid) {
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

TEST(A2D2_to_ROS, axes_are_valid) {
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

TEST(A2D2_to_ROS, get_orthonormal_basis) {
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
    EXPECT_TRUE(B.isApprox(B_expected, EPS));
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
    EXPECT_TRUE(B.isApprox(B_expected, EPS))
        << "B: " << B << "\nDid not match B_expected: " << B_expected;
  }
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, build_ego_shape_msg) {
  {
    const auto x_min = 0.0;
    const auto x_max = 1.0;
    const auto y_min = 0.0;
    const auto y_max = 1.0;
    const auto z_min = 0.0;
    const auto z_max = 1.0;
    const auto msg =
        build_ego_shape_msg(x_min, x_max, y_min, y_max, z_min, z_max);
    EXPECT_EQ(msg.type, shape_msgs::SolidPrimitive::BOX);
    ASSERT_EQ(msg.dimensions.size(), 3);
    EXPECT_EQ(msg.dimensions[shape_msgs::SolidPrimitive::BOX_X], 1.0);
    EXPECT_EQ(msg.dimensions[shape_msgs::SolidPrimitive::BOX_Y], 1.0);
    EXPECT_EQ(msg.dimensions[shape_msgs::SolidPrimitive::BOX_Z], 1.0);
  }

  {
    const auto x_min = -1.0;
    const auto x_max = 1.0;
    const auto y_min = -1.0;
    const auto y_max = 1.0;
    const auto z_min = -1.0;
    const auto z_max = 1.0;
    const auto msg =
        build_ego_shape_msg(x_min, x_max, y_min, y_max, z_min, z_max);
    EXPECT_EQ(msg.type, shape_msgs::SolidPrimitive::BOX);
    ASSERT_EQ(msg.dimensions.size(), 3);
    EXPECT_EQ(msg.dimensions[shape_msgs::SolidPrimitive::BOX_X], 2.0);
    EXPECT_EQ(msg.dimensions[shape_msgs::SolidPrimitive::BOX_Y], 2.0);
    EXPECT_EQ(msg.dimensions[shape_msgs::SolidPrimitive::BOX_Z], 2.0);
  }
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, verify_ego_bbox_params) {
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

TEST(A2D2_to_ROS, camera_name_from_lidar_name) {
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

TEST(A2D2_to_ROS, frame_from_filename) {
  {
    const auto frame_expected = "frontcenter";
    const auto filename =
        "/home/maeve/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/lidar/"
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
    const auto frame_expected = "";
    const auto filename = "20190401145936_lidar_rearleft_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }

  {
    const auto frame_expected = "";
    const auto filename = "20190401145936_lidar_rearright_000000080";
    const auto frame = frame_from_filename(filename);
    EXPECT_EQ(frame_expected, frame);
  }
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, flatten_2d_index) {
  /**
   * 0  1  2  3  4
   * 5  6  7  8  9
   * 10 11 12 13 14
   */
  constexpr auto WIDTH = 5;

  {
    constexpr auto idx = 8;
    constexpr auto row = 1;
    constexpr auto col = 3;
    EXPECT_EQ(idx, flatten_2d_index(WIDTH, row, col));
  }

  {
    constexpr auto idx = 11;
    constexpr auto row = 2;
    constexpr auto col = 1;
    EXPECT_EQ(idx, flatten_2d_index(WIDTH, row, col));
  }

  {
    constexpr auto idx = 0;
    constexpr auto row = 0;
    constexpr auto col = 0;
    EXPECT_EQ(idx, flatten_2d_index(WIDTH, row, col));
  }

  {
    constexpr auto idx = 14;
    constexpr auto row = 2;
    constexpr auto col = 4;
    EXPECT_EQ(idx, flatten_2d_index(WIDTH, row, col));
  }

  {
    constexpr auto idx = 5;
    constexpr auto row = 1;
    constexpr auto col = 0;
    EXPECT_EQ(idx, flatten_2d_index(WIDTH, row, col));
  }

  {
    constexpr auto idx = 1;
    constexpr auto row = 0;
    constexpr auto col = 1;
    EXPECT_EQ(idx, flatten_2d_index(WIDTH, row, col));
  }
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, a2d2_timestamp_to_ros_time) {
  constexpr auto TIME = static_cast<uint64_t>(1554122338652775);
  const auto ros_time = a2d2_timestamp_to_ros_time(TIME);
  const auto nsecs = ros_time.toNSec();
  const auto test_secs = (nsecs / ONE_THOUSAND);
  EXPECT_EQ(test_secs, TIME);
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, valid_ros_timestamp) {
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

TEST(A2D2_to_ROS, get_unit_enum) {
  EXPECT_EQ(get_unit_enum("null"), Units::null);
  EXPECT_EQ(get_unit_enum("Unit_Bar"), Units::Unit_Bar);
  EXPECT_EQ(get_unit_enum("Unit_PerCent"), Units::Unit_PerCent);
  EXPECT_EQ(get_unit_enum("Unit_DegreOfArc"), Units::Unit_DegreOfArc);
  EXPECT_EQ(get_unit_enum("Unit_KiloMeterPerHour"),
            Units::Unit_KiloMeterPerHour);
  EXPECT_EQ(get_unit_enum("Unit_MeterPerSeconSquar"),
            Units::Unit_MeterPerSeconSquar);
  EXPECT_EQ(get_unit_enum("Unit_DegreOfArcPerSecon"),
            Units::Unit_DegreOfArcPerSecon);
  EXPECT_EQ(get_unit_enum(""), Units::UNKNOWN);
}

//------------------------------------------------------------------------------

TEST(A2D2_to_ROS, to_ros_units) {
  const auto deg_to_rad = [](double deg) { return (deg * (M_PI / 180.0)); };
  const auto kph_to_mps = [](double kph) {
    return (kph * (1000.0 / (60.0 * 60.0)));
  };
  const auto percent_to_unit = [](double percent) { return (percent / 100.0); };

  {
    constexpr auto val = 36.37;
    EXPECT_EQ(to_ros_units("Unit_DegreOfArc", val), deg_to_rad(val));
    EXPECT_EQ(to_ros_units("Unit_DegreOfArcPerSecon", val), deg_to_rad(val));
  }

  {
    constexpr auto val = 15.0;
    EXPECT_EQ(to_ros_units("Unit_KiloMeterPerHour", val), kph_to_mps(val));
  }

  {
    constexpr auto val = 99.1;
    EXPECT_EQ(to_ros_units("Unit_PerCent", val), percent_to_unit(val));
  }

  {
    constexpr auto val = 13.05;
    EXPECT_EQ(to_ros_units("null", val), val);
    EXPECT_EQ(to_ros_units("Unit_Bar", val), val);
    EXPECT_EQ(to_ros_units("Unit_MeterPerSeconSquar", val), val);
  }

  {
    constexpr auto val = 0.0;
    const auto ros_units = to_ros_units("UNKNOWN", val);
    EXPECT_TRUE(std::isnan(ros_units));
  }
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros

