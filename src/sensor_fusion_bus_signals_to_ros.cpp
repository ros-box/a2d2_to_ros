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
#include <limits>
#include <set>
#include <sstream>
#include <tuple>
#include <unordered_map>

#include <boost/filesystem/convenience.hpp>  // TODO(jeff): use std::filesystem in C++17
#include <boost/optional.hpp>
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#ifdef USE_FLOAT64
#include <std_msgs/Float64.h>
#else
#include <std_msgs/Float32.h>
#endif
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/schema.h"
#include "rapidjson/stringbuffer.h"

#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"
#include "a2d2_to_ros/log_build_options.hpp"
#include "a2d2_to_ros/logging.hpp"

namespace {
namespace a2d2 = a2d2_to_ros;
namespace po = boost::program_options;
}  // namespace

typedef std::set<a2d2::DataPair, a2d2::DataPairTimeComparator> DataPairSet;
typedef std::unordered_map<std::string, std::tuple<std::string, DataPairSet>>
    DataPairMap;

///
/// Program constants and defaults.
///

static constexpr auto _PROGRAM_OPTIONS_LINE_LENGTH = 120u;
static constexpr auto _INCLUDE_ORIGINAL = false;
static constexpr auto _INCLUDE_CONVERTED = true;
static constexpr auto _INCLUDE_CLOCK_TOPIC = false;
static constexpr auto _CLOCK_TOPIC = "/clock";
static constexpr auto _BUS_FRAME_NAME = "wheels";  // TODO(jeff): is this right?
static constexpr auto _OUTPUT_PATH = ".";
static constexpr auto _DATASET_NAMESPACE = "/a2d2";
static constexpr auto _ORIGINAL_VALUE_TOPIC = "original_value";
static constexpr auto _ORIGINAL_UNITS_TOPIC = "original_units";
static constexpr auto _VALUE_TOPIC = "value";
static constexpr auto _HEADER_TOPC = "header";
static constexpr auto _MIN_TIME_OFFSET = 0.0;
static constexpr auto _DURATION = std::numeric_limits<double>::max();

int main(int argc, char* argv[]) {
  BUILD_INFO;  // just write to log what build options were specified

  ///
  /// Set up command line arguments
  ///

  boost::optional<std::string> schema_path_opt;
  boost::optional<std::string> json_path_opt;

  po::options_description desc(
      "Convert sequential bus signal data to rosbag for the A2D2 Sensor Fusion "
      "data set. See README.md for details.\nAvailable options are listed "
      "below. Arguments without default values are required",
      _PROGRAM_OPTIONS_LINE_LENGTH);
  desc.add_options()("help,h", "Print help and exit.")(
      "schema-path,s", po::value(&schema_path_opt)->required(),
      "Path to the JSON schema.")("json-path,j",
                                  po::value(&json_path_opt)->required(),
                                  "Path to the JSON data set file.")(
      "min-time-offset,m", po::value<double>()->default_value(_MIN_TIME_OFFSET),
      "Optional: Seconds to skip ahead in the data before starting the bag.")(
      "duration,d", po::value<double>()->default_value(_DURATION),
      "Optional: Seconds after min-time-offset to include in bag file.")(
      "output-path,o", po::value<std::string>()->default_value(_OUTPUT_PATH),
      "Optional: Path for the output bag file.")(
      "include-clock-topic,c",
      po::value<bool>()->default_value(_INCLUDE_CLOCK_TOPIC),
      "Optional: Use timestamps from the data to write a /clock topic.")(
      "include-original-values,v",
      po::value<bool>()->default_value(_INCLUDE_ORIGINAL),
      "Optional: Include data set values in their original units.")(
      "include-converted-values,r",
      po::value<bool>()->default_value(_INCLUDE_CONVERTED),
      "Optional: Include data set values converted to ROS standard units.");

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

  const auto schema_path = *schema_path_opt;
  const auto json_path = *json_path_opt;
  const auto output_path = vm["output-path"].as<std::string>();
  const auto include_original = vm["include-original-values"].as<bool>();
  const auto include_converted = vm["include-converted-values"].as<bool>();
  const auto include_clock_topic = vm["include-clock-topic"].as<bool>();
  const auto min_time_offset = vm["min-time-offset"].as<double>();
  const auto duration = vm["duration"].as<double>();

  // TODO(jeff): remove trailing slashes from paths

  const auto valid_min_offset =
      (std::isfinite(min_time_offset) && (min_time_offset >= 0.0));
  const auto valid_duration = (std::isfinite(duration) && (duration >= 0.0));
  if (!valid_min_offset || !valid_duration) {
    X_FATAL(
        "Time constraints {min-time-offset: "
        << min_time_offset << ", duration: " << duration
        << "} are not valid. They must be finite, real valued, and >= 0.0.");
    return EXIT_FAILURE;
  }

  const auto file_basename = boost::filesystem::basename(json_path);
  const auto topic_prefix =
      (std::string(_DATASET_NAMESPACE) + "/" + file_basename);

  ///
  /// Get the JSON schema for the data set
  ///

  rapidjson::Document d_schema;
  {
    // get schema file string
    const auto schema_string = a2d2::get_file_as_string(schema_path);
    if (schema_string.empty()) {
      X_FATAL("'" << schema_path << "' failed to open or is empty.");
      return EXIT_FAILURE;
    }

    if (d_schema.Parse(schema_string.c_str()).HasParseError()) {
      fprintf(stderr, "\nError(offset %u): %s\n",
              static_cast<unsigned>(d_schema.GetErrorOffset()),
              rapidjson::GetParseError_En(d_schema.GetParseError()));
      return EXIT_FAILURE;
    }
  }
  rapidjson::SchemaDocument schema(d_schema);

  ///
  /// Get the JSON data set
  ///

  rapidjson::Document d_json;
  {
    // get json file string
    const auto json_string = a2d2::get_file_as_string(json_path);
    if (json_string.empty()) {
      X_FATAL("'" << json_path << "' failed to open or is empty.");
      return EXIT_FAILURE;
    }

    if (d_json.Parse(json_string.c_str()).HasParseError()) {
      X_FATAL("Error(offset "
              << static_cast<unsigned>(d_json.GetErrorOffset())
              << "): " << rapidjson::GetParseError_En(d_json.GetParseError()));
      return EXIT_FAILURE;
    }
  }

  X_INFO("Loaded and parsed schema and data set successfully.");

  ///
  /// Validate the data set against the schema
  ///

  rapidjson::SchemaValidator validator(schema);
  if (!d_json.Accept(validator)) {
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
  }

  X_INFO("JSON data validated against schema, ready to convert.");

  ///
  /// Ensure that the schema has necessary information for the bag file
  ///

  {
    const auto has_required = d_schema.HasMember("required");
    const auto is_array = d_schema["required"].IsArray();
    const auto is_empty = (is_array && d_schema["required"].GetArray().Empty());
    if (!has_required || !is_array || is_empty) {
      X_FATAL(
          "Schema either does not have a 'required' member, or the member is "
          "not "
          "an array, or the array is empty: HasMember('required'): "
          << has_required << ", isArray(): " << is_array
          << ", Empty(): " << is_empty);
      return EXIT_FAILURE;
    }

    const rapidjson::Value& r = d_schema["required"];
    for (rapidjson::SizeType idx = 0; idx < r.Size(); ++idx) {
      if (!r[idx].IsString()) {
        X_FATAL("Required field at index " << idx << " is not a string type.");
        return EXIT_FAILURE;
      }

      const auto name = std::string(r[idx].GetString());
      if (name.empty()) {
        X_FATAL("Required field name at index " << idx << " is empty.");
        return EXIT_FAILURE;
      }
    }
  }

  ///
  /// Get the field names from schema, retrieve data from DOM, write to bag.
  /// This loop does not do error checking on the DOM because it should already
  /// have been validated by the schema.
  ///

  rosbag::Bag bag;
  std::set<ros::Time> stamps;
  const auto bag_name = output_path + "/" + file_basename + ".bag";
  bag.open(bag_name, rosbag::bagmode::Write);
  const rapidjson::Value& r = d_schema["required"];
  for (rapidjson::SizeType idx = 0; idx < r.Size(); ++idx) {
    const auto name = std::string(r[idx].GetString());
    X_INFO("Converting " << name << "...");

    const rapidjson::Value& obj = d_json[name.c_str()].GetObject();
    const rapidjson::Value& values = obj["values"];
    const auto units = obj["unit"].IsNull() ? "null" : obj["unit"].GetString();

    // publish data for each of the values in this field
    auto no_units_yet = true;
    boost::optional<ros::Time> first_time;
    for (rapidjson::SizeType idx = 0; idx < values.Size(); ++idx) {
      const rapidjson::Value& t_v = values[idx];
      const auto time = t_v[static_cast<rapidjson::SizeType>(0)].GetUint64();
      if (!a2d2::valid_ros_timestamp(time)) {
        X_FATAL("Timestamp "
                << time
                << " has unsupported magnitude: ROS does not support "
                   "timestamps on or after 4294967296000000 "
                   "(Sunday, February 7, 2106 6:28:16 AM GMT)\nCall "
                   "Zager and Evans for details.");
        return EXIT_FAILURE;
      }

      const auto value = t_v[static_cast<rapidjson::SizeType>(1)].GetDouble();
      const auto data = a2d2::DataPair::build(value, time, _BUS_FRAME_NAME);

      const auto& stamp = data.header.stamp;
      if (!first_time) {
        first_time = stamp;
      }

      const auto time_since_begin = (stamp - *first_time).toSec();
      if (time_since_begin < min_time_offset) {
        continue;
      }

      const auto recorded_duration = (time_since_begin - min_time_offset);
      if (recorded_duration > duration) {
        break;
      }

      bag.write(topic_prefix + "/" + name + "/" + _HEADER_TOPC, stamp,
                data.header);
      if (include_original) {
        bag.write(topic_prefix + "/" + name + "/" + _ORIGINAL_VALUE_TOPIC,
                  stamp, data.value);

        if (no_units_yet) {
          std_msgs::String units_msg;
          units_msg.data = units;

          bag.write(topic_prefix + "/" + name + "/" + _ORIGINAL_UNITS_TOPIC,
                    stamp, units_msg);
          no_units_yet = false;
        }
      }

      if (include_converted) {
        const auto ros_value = a2d2::to_ros_units(units, data.value.data);
        a2d2::DataPair::value_type ros_value_msg;
        ros_value_msg.data = ros_value;
        bag.write(topic_prefix + "/" + name + "/" + _VALUE_TOPIC, stamp,
                  ros_value_msg);
      }

      if (include_clock_topic) {
        stamps.insert(stamp);
      }
    }
  }

  ///
  /// Write a clock message for every unique timestamp in the data set
  ///

  if (include_clock_topic) {
    X_INFO("Adding " << _CLOCK_TOPIC << " topic...");
  }
  for (const auto& stamp : stamps) {
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = stamp;
    bag.write(_CLOCK_TOPIC, stamp, clock_msg);
  }

  ///
  /// Finish the bag and exit
  ///

  bag.close();
  return EXIT_SUCCESS;
}
