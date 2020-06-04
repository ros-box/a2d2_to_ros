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
#include "a2d2_to_ros/checks.hpp"

#include <cmath>

#include "a2d2_to_ros/conversions.hpp"

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

bool vector_is_valid(const Eigen::Vector3d& v) {
  return std::isfinite(v.norm());
}

//------------------------------------------------------------------------------

bool axis_is_valid(const Eigen::Vector3d& axis, double epsilon) {
  return (vector_is_valid(axis) && (axis.norm() > epsilon));
}

//------------------------------------------------------------------------------

bool axes_are_valid(const Eigen::Vector3d& axis1, const Eigen::Vector3d& axis2,
                    double epsilon) {
  const auto axis1_valid = axis_is_valid(axis1, epsilon);
  const auto axis2_valid = axis_is_valid(axis2, epsilon);
  const auto axes_not_equal = !axis1.isApprox(axis2, epsilon);
  return (axis1_valid && axis2_valid && axes_not_equal);
}

//------------------------------------------------------------------------------

bool valid_ros_timestamp(uint64_t time) {
  const auto secs = microseconds_to_seconds(time);
  const auto boundary =
      static_cast<uint64_t>(std::numeric_limits<uint32_t>::max());
  return (secs <= boundary);
}

//------------------------------------------------------------------------------

bool verify_ego_bbox_params(double x_min, double x_max, double y_min,
                            double y_max, double z_min, double z_max) {
  const std::array<double, 6> vals = {x_min, x_max, y_min, y_max, z_min, z_max};
  const auto all_finite =
      std::all_of(std::begin(vals), std::end(vals),
                  [](double v) { return std::isfinite(v); });

  const auto x_ordered = (x_min < x_max);
  const auto y_ordered = (y_min < y_max);
  const auto z_ordered = (z_min < z_max);
  const auto all_ordered = (x_ordered && y_ordered && z_ordered);

  return (all_finite && all_ordered);
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros
