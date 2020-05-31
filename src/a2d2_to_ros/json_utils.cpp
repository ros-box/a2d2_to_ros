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
#include "a2d2_to_ros/json_utils.hpp"

#include <sstream>

#include "a2d2_to_ros/file_utils.hpp"
#include "a2d2_to_ros/transform_utils.hpp"

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

std::string get_validator_error_string(
    const rapidjson::SchemaValidator& validator) {
  rapidjson::StringBuffer sb;
  validator.GetInvalidSchemaPointer().StringifyUriFragment(sb);
  std::stringstream ss;
  ss << "\nInvalid schema: " << sb.GetString() << "\n";
  ss << "Invalid keyword: " << validator.GetInvalidSchemaKeyword() << "\n";
  sb.Clear();
  validator.GetInvalidDocumentPointer().StringifyUriFragment(sb);
  ss << "Invalid document: " << sb.GetString() << "\n";
  return ss.str();
}

//------------------------------------------------------------------------------

boost::optional<rapidjson::Document> get_rapidjson_dom(
    const std::string& path) {
  rapidjson::Document d;

  // get schema file string
  const auto schema_string = get_file_as_string(path);
  if (schema_string.empty()) {
    //      X_FATAL("'" << path << "' failed to open or is empty.");
    return boost::none;
  }

  if (d.Parse(schema_string.c_str()).HasParseError()) {
    /*
    fprintf(stderr, "\nError(offset %u): %s\n",
            static_cast<unsigned>(d_schema.GetErrorOffset()),
            rapidjson::GetParseError_En(d_schema.GetParseError()));
            */
    return boost::none;
  }

  return d;
}

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
