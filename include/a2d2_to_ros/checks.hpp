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
#ifndef A2D2_TO_ROS__CHECKS_HPP_
#define A2D2_TO_ROS__CHECKS_HPP_

#include <Eigen/Core>

namespace a2d2_to_ros {

/**
 * @brief Check that a value is > 0
 */
template <typename T>
bool strictly_positive(const T& val) {
  return (val > static_cast<T>(0));
}

/**
 * @brief Check that a value is > 0
 */
template <typename T>
bool strictly_non_negative(const T& val) {
  const auto is_strictly_positive = strictly_positive(val);
  const auto is_zero = (val == static_cast<T>(0));
  return (is_strictly_positive || is_zero);
}

/**
 * @brief Verify that the vector is valid.
 * @return true iff the vector is finite and real-valued
 */
bool vector_is_valid(const Eigen::Vector3d& v);

/**
 * @brief Verify that the axis is valid.
 * @return true iff the axis is a valid vector and is non-singular w.r.t.
 * epsilon
 */
bool axis_is_valid(const Eigen::Vector3d& axis, double epsilon);

/**
 * @brief Verify that axes are valid and unique.
 * @note This is a convenience method to run axis_is_valid on each input axis
 * and then additionally check that they are unique.
 * @return true iff the axes are both valid according to 'axis_is_valid' and
 * differ by at least magnitude epsilon.
 */
bool axes_are_valid(const Eigen::Vector3d& axis1, const Eigen::Vector3d& axis2,
                    double epsilon);

/**
 * @brief Test whether a given timestamp is covertible to ROS time.
 * @return True iff converting time to ros::Time will not cause overflow.
 */
bool valid_ros_timestamp(uint64_t time);

/**
 * @brief Check whether the ego bbox parameters makes sense.
 * @return true iff the values are valid, i.e., real valued, finite, ordered,
 * and the bbox has non-zero measure.
 */
bool verify_ego_bbox_params(double x_min, double x_max, double y_min,
                            double y_max, double z_min, double z_max);

}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__CHECKS_HPP_
