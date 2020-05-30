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
#ifndef A2D2_TO_ROS__JSON_UTILS_HPP_
#define A2D2_TO_ROS__JSON_UTILS_HPP_

#include <Eigen/Core>

#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/schema.h"
#include "rapidjson/stringbuffer.h"

namespace a2d2_to_ros {

/**
 * @brief Utility to convert an axis from a JSON DOM to Eigen.
 * @pre The rapidjson value is valid ['view']['(x|y)-axis'] according to the
 * schema.
 */
Eigen::Vector3d json_axis_to_eigen_vector(const rapidjson::Value& json_axis);

/**
 * @brief Utility to retrieve an orthonormal basis from a JSON doc.
 * @pre The doc must validate according to the schema
 */
Eigen::Matrix3d json_axes_to_eigen_basis(const rapidjson::Document& d,
                                         const std::string& sensor,
                                         const std::string& frame,
                                         double epsilon);

/**
 * @brief Utility to retrieve a basis origin from a JSON doc.
 * @pre The doc must validate according to the schema
 */
Eigen::Vector3d json_origin_to_eigen_vector(const rapidjson::Document& d,
                                            const std::string& sensor,
                                            const std::string& frame);

}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__JSON_UTILS_HPP_
