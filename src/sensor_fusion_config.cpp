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
#include <algorithm>
#include <set>

#include <boost/filesystem/convenience.hpp>  // TODO(jeff): use std::filesystem in C++17
#include <boost/optional.hpp>
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/schema.h"
#include "rapidjson/stringbuffer.h"

#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"
#include "a2d2_to_ros/log_build_options.hpp"
#include "a2d2_to_ros/logging.hpp"
#include "ros_cnpy/cnpy.h"

///
/// Program constants and defaults.
///

static constexpr auto EPS = 1e-8;
static constexpr auto _PROGRAM_OPTIONS_LINE_LENGTH = 120u;
static constexpr auto _CLOCK_TOPIC = "/clock";
static constexpr auto _OUTPUT_PATH = ".";
static constexpr auto _DATASET_NAMESPACE = "/a2d2";
static constexpr auto _INCLUDE_DEPTH_MAP = false;
static constexpr auto _VERBOSE = false;

///
/// Executable specific stuff
///

#define VERIFY_BASIS(basis, sensors, frame)                                 \
  {                                                                         \
    if (basis.isZero(0.0)) {                                                \
      X_FATAL(                                                              \
          "Basis for "                                                      \
          << sensors << "::" << frame                                       \
          << " cannot be constructed. Check that the X/Y axes are valid."); \
      return EXIT_FAILURE;                                                  \
    }                                                                       \
  }

namespace {
namespace a2d2 = a2d2_to_ros;
namespace po = boost::program_options;

/**
 * @brief Utility to convert an axis from a JSON DOM to Eigen.
 * @pre The rapidjson value is valid ['view']['(x|y)-axis'] according to the
 * schema.
 */
Eigen::Vector3d json_axis_to_eigen_vector(const rapidjson::Value& json_axis) {
  constexpr auto X_IDX = static_cast<rapidjson::SizeType>(0);
  constexpr auto Y_IDX = static_cast<rapidjson::SizeType>(1);
  constexpr auto Z_IDX = static_cast<rapidjson::SizeType>(2);
  return Eigen::Vector3d(json_axis[X_IDX].GetDouble(),
                         json_axis[Y_IDX].GetDouble(),
                         json_axis[Z_IDX].GetDouble());
}

/**
 * @brief Utility to retrieve an orthonormal basis from a JSON doc.
 * @pre The doc must validate according to the schema
 */
Eigen::Matrix3d json_axes_to_eigen_basis(const rapidjson::Document& d,
                                         const std::string& sensor,
                                         const std::string& frame) {
  const rapidjson::Value& view = d[sensor.c_str()][frame.c_str()]["view"];

  const Eigen::Vector3d x_axis = json_axis_to_eigen_vector(view["x-axis"]);
  const Eigen::Vector3d y_axis = json_axis_to_eigen_vector(view["y-axis"]);
  return a2d2::get_orthonormal_basis(x_axis, y_axis, EPS);
}
}  // namespace

//------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  BUILD_INFO;  // just write to log what build options were specified

  ///
  /// Set up command line arguments
  ///

  // TODO(jeff): rename "reflectance" to "intensity" assuming that's what it is
  boost::optional<std::string> sensor_config_path_opt;
  boost::optional<std::string> sensor_config_schema_path_opt;
  po::options_description desc(
      "Publish the vehicle box model and tf tree for the vehicle sensor "
      "configuration.",
      _PROGRAM_OPTIONS_LINE_LENGTH);
  desc.add_options()("help,h", "Print help and exit.")(
      "sensor-config-path,c", po::value(&sensor_config_path_opt)->required(),
      "Path to the JSON for vehicle/sensor config.")(
      "sensor-config-schema-path,s",
      po::value(&sensor_config_schema_path_opt)->required(),
      "Path to the JSON schema for the vehicle/sensor config.")(
      "output-path,o", po::value<std::string>()->default_value(_OUTPUT_PATH),
      "Optional: Path for the output bag file.");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  const auto help_requested = vm.count("help");
  if (help_requested) {
    std::cout << desc << "\n";
    return EXIT_SUCCESS;
  }

  try {
    po::notify(vm);
  } catch (boost::program_options::required_option& e) {
    std::cerr << "Ensure that all required options are specified: " << e.what()
              << "\n\n";
    std::cerr << desc << "\n";
    return EXIT_FAILURE;
  }

  ///
  /// Get commandline parameters
  ///

  const auto sensor_config_path = *sensor_config_path_opt;
  const auto sensor_config_schema_path = *sensor_config_schema_path_opt;
  const auto output_path = vm["output-path"].as<std::string>();

  ///
  /// Get the JSON for vehicle/sensor config
  ///

  rapidjson::Document sensor_config_d;
  {
    const auto sensor_config_json_string =
        a2d2::get_json_file_as_string(sensor_config_path);
    if (sensor_config_json_string.empty()) {
      X_FATAL("'" << sensor_config_path << "' failed to open or is empty.");
      return EXIT_FAILURE;
    }

    if (sensor_config_d.Parse(sensor_config_json_string.c_str())
            .HasParseError()) {
      X_FATAL("Error(offset "
              << static_cast<unsigned>(sensor_config_d.GetErrorOffset())
              << "): "
              << rapidjson::GetParseError_En(sensor_config_d.GetParseError()));
      return EXIT_FAILURE;
    }
  }

  ///
  /// Get the JSON schema for the config JSON
  ///

  rapidjson::Document schema_d;
  {
    // get schema file string
    const auto schema_string =
        a2d2::get_json_file_as_string(sensor_config_schema_path);
    if (schema_string.empty()) {
      X_FATAL("'" << sensor_config_schema_path
                  << "' failed to open or is empty.");
      return EXIT_FAILURE;
    }

    if (schema_d.Parse(schema_string.c_str()).HasParseError()) {
      fprintf(stderr, "\nError(offset %u): %s\n",
              static_cast<unsigned>(schema_d.GetErrorOffset()),
              rapidjson::GetParseError_En(schema_d.GetParseError()));
      return EXIT_FAILURE;
    }
  }
  rapidjson::SchemaDocument config_schema(schema_d);

  ///
  /// Validate the data set against the schema
  ///

  {
    rapidjson::SchemaValidator validator(config_schema);
    if (!sensor_config_d.Accept(validator)) {
      rapidjson::StringBuffer sb;
      validator.GetInvalidSchemaPointer().StringifyUriFragment(sb);
      std::stringstream ss;
      ss << "\nInvalid schema: " << sb.GetString() << "\n";
      ss << "Invalid keyword: " << validator.GetInvalidSchemaKeyword() << "\n";
      sb.Clear();
      validator.GetInvalidDocumentPointer().StringifyUriFragment(sb);
      ss << "Invalid document: " << sb.GetString() << "\n";
      X_FATAL(ss.str());
      return EXIT_FAILURE;
    } else {
      X_INFO("Validated: " << sensor_config_schema_path);
    }
  }

  ///
  /// Build ego vehicle shape message
  ///

  const rapidjson::Value& ego_dims =
      sensor_config_d["vehicle"]["ego-dimensions"];
  const rapidjson::Value& x_dims = ego_dims["x-range"];
  const rapidjson::Value& y_dims = ego_dims["y-range"];
  const rapidjson::Value& z_dims = ego_dims["z-range"];

  constexpr auto MIN_IDX = static_cast<rapidjson::SizeType>(0);
  constexpr auto MAX_IDX = static_cast<rapidjson::SizeType>(1);
  const auto x_min = x_dims[MIN_IDX].GetDouble();
  const auto x_max = x_dims[MAX_IDX].GetDouble();
  const auto y_min = y_dims[MIN_IDX].GetDouble();
  const auto y_max = y_dims[MAX_IDX].GetDouble();
  const auto z_min = z_dims[MIN_IDX].GetDouble();
  const auto z_max = z_dims[MAX_IDX].GetDouble();

  const auto ego_bbox_valid =
      a2d2::verify_ego_bbox_params(x_min, x_max, y_min, y_max, z_min, z_max);
  if (!ego_bbox_valid) {
    X_FATAL(
        "Ego bounding box parameters are invalid. They must be finite, "
        "real-valued, and ordered: x: ["
        << x_min << ", " << x_max << "], y: [" << y_min << ", " << y_max
        << "], z: [" << z_min << ", " << z_max << "]");
    return EXIT_FAILURE;
  }

  const auto ego_shape_msg =
      a2d2::build_ego_shape_msg(x_min, x_max, y_min, y_max, z_min, z_max);

  ///
  /// Get lidar poses
  ///

  {
    Eigen::Matrix3d lidar_front_center_basis;
    const auto sensors = "lidars";
    const auto frame = "front_center";
    lidar_front_center_basis =
        json_axes_to_eigen_basis(sensor_config_d, sensors, frame);
    VERIFY_BASIS(lidar_front_center_basis, sensors, frame);
  }

  {
    Eigen::Matrix3d lidar_front_left_basis;
    const auto sensors = "lidars";
    const auto frame = "front_left";
    lidar_front_left_basis =
        json_axes_to_eigen_basis(sensor_config_d, sensors, frame);
    VERIFY_BASIS(lidar_front_left_basis, sensors, frame);
  }

  {
    Eigen::Matrix3d lidar_front_right_basis;
    const auto sensors = "lidars";
    const auto frame = "front_right";
    lidar_front_right_basis =
        json_axes_to_eigen_basis(sensor_config_d, sensors, frame);
    VERIFY_BASIS(lidar_front_right_basis, sensors, frame);
  }

  {
    Eigen::Matrix3d lidar_rear_left_basis;
    const auto sensors = "lidars";
    const auto frame = "rear_left";
    lidar_rear_left_basis =
        json_axes_to_eigen_basis(sensor_config_d, sensors, frame);
    VERIFY_BASIS(lidar_rear_left_basis, sensors, frame);
  }

  {
    Eigen::Matrix3d lidar_rear_right_basis;
    const auto sensors = "lidars";
    const auto frame = "rear_right";
    lidar_rear_right_basis =
        json_axes_to_eigen_basis(sensor_config_d, sensors, frame);
    VERIFY_BASIS(lidar_rear_right_basis, sensors, frame);
  }

  ///
  /// Get camera poses
  ///

  return EXIT_SUCCESS;
}
