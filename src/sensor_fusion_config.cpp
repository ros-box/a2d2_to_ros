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

namespace {
namespace a2d2 = a2d2_to_ros;
namespace po = boost::program_options;
}  // namespace

///
/// Program constants and defaults.
///

static constexpr auto _PROGRAM_OPTIONS_LINE_LENGTH = 120u;
static constexpr auto _CLOCK_TOPIC = "/clock";
static constexpr auto _OUTPUT_PATH = ".";
static constexpr auto _DATASET_NAMESPACE = "/a2d2";
static constexpr auto _INCLUDE_DEPTH_MAP = false;
static constexpr auto _VERBOSE = false;

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

  return EXIT_SUCCESS;
}
