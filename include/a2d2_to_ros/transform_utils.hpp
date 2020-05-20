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
#ifndef A2D2_TO_ROS__TRANSFORM_UTILS_HPP_
#define A2D2_TO_ROS__TRANSFORM_UTILS_HPP_

#include <Eigen/Geometry>

namespace a2d2_to_ros {

/**
 * @brief Given an X and a Y axis in 3-space, compute a right-handed orthonormal
 * basis with Z orthogonal to the input X and Y, and a new Y orthogonal to X and
 * the computed Z.
 * @pre Vectors X and Y are unique and non-singular.
 * @return A 3x3 matrix whose columns [X, Y, Z] form a right-handed orthonormal
 * basis. If input X and Y are not valid, a zero matrix is returned.
 */
Eigen::Matrix3d get_orthonormal_basis(const Eigen::Vector3d& X,
                                      const Eigen::Vector3d& Y, double epsilon);

/**
 * @brief Compute transform between sensor and global coordinates.
 */
Eigen::Affine3d Tx_global_sensor(const Eigen::Matrix3d& basis,
                                 const Eigen::Vector3d& origin);

}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__TRANSFORM_UTILS_HPP_
