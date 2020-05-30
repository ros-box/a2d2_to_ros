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
#include "a2d2_to_ros/sensor_config_utils.hpp"

#include "a2d2_to_ros/transform_utils.hpp"

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

Eigen::Vector3d json_axis_to_eigen_vector(const rapidjson::Value& json_axis) {
  constexpr auto X_IDX = static_cast<rapidjson::SizeType>(0);
  constexpr auto Y_IDX = static_cast<rapidjson::SizeType>(1);
  constexpr auto Z_IDX = static_cast<rapidjson::SizeType>(2);
  return Eigen::Vector3d(json_axis[X_IDX].GetDouble(),
                         json_axis[Y_IDX].GetDouble(),
                         json_axis[Z_IDX].GetDouble());
}

//------------------------------------------------------------------------------

Eigen::Matrix3d json_axes_to_eigen_basis(const rapidjson::Document& d,
                                         const std::string& sensor,
                                         const std::string& frame,
                                         double epsilon) {
  const rapidjson::Value& view = d[sensor.c_str()][frame.c_str()]["view"];

  const Eigen::Vector3d x_axis = json_axis_to_eigen_vector(view["x-axis"]);
  const Eigen::Vector3d y_axis = json_axis_to_eigen_vector(view["y-axis"]);
  return get_orthonormal_basis(x_axis, y_axis, epsilon);
}

//------------------------------------------------------------------------------

Eigen::Vector3d json_origin_to_eigen_vector(const rapidjson::Document& d,
                                            const std::string& sensor,
                                            const std::string& frame) {
  return json_axis_to_eigen_vector(
      d[sensor.c_str()][frame.c_str()]["view"]["origin"]);
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros
