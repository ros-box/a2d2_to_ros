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

#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/schema.h"
#include "rapidjson/stringbuffer.h"

// uncomment this define to log warnings and errors
#define _ENABLE_A2D2_ROS_LOGGING_
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"

typedef std::set<a2d2_to_ros::DataPair, a2d2_to_ros::DataPairTimeComparator>
    DataPairSet;
typedef std::unordered_map<std::string, std::tuple<std::string, DataPairSet>>
    DataPairMap;

///
/// Program constants and defaults.
///

static constexpr auto _PROGRAM_OPTIONS_LINE_LENGTH = 120u;
static constexpr auto _INCLUDE_ORIGINAL = true;
static constexpr auto _INCLUDE_CONVERTED = true;
static constexpr auto _CLOCK_TOPIC = "/clock";
static constexpr auto _BUS_FRAME_NAME = "bus";
static constexpr auto _OUTPUT_PATH = ".";
static constexpr auto _DATASET_NAMESPACE = "/a2d2";
static constexpr auto _ORIGINAL_VALUE_TOPIC = "original_value";
static constexpr auto _ORIGINAL_UNITS_TOPIC = "original_units";
static constexpr auto _VALUE_TOPIC = "value";
static constexpr auto _HEADER_TOPC = "header";

namespace {
namespace po = boost::program_options;
}  // namespace

int main(int argc, char* argv[]) {
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
      "output-path,o", po::value<std::string>()->default_value(_OUTPUT_PATH),
      "Optional: Path for the output bag file.")(
      "bus-frame-name,b",
      po::value<std::string>()->default_value(_BUS_FRAME_NAME),
      "Optional: Frame name to use for bus signals.")(
      "include-original-values,v",
      po::value<bool>()->default_value(_INCLUDE_ORIGINAL),
      "Optional: Include the original data set values in their original "
      "units.")(
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
  const auto bus_frame_name = vm["bus-frame-name"].as<std::string>();

  const auto file_basename = boost::filesystem::basename(json_path);
  const auto topic_prefix =
      (std::string(_DATASET_NAMESPACE) + "/" + file_basename);

  ///
  /// Get the JSON schema for the data set
  ///

  rapidjson::Document d_schema;
  {
    // get schema file string
    const auto schema_string =
        a2d2_to_ros::get_json_file_as_string(schema_path);
    if (schema_string.empty()) {
      ROS_FATAL_STREAM("'" << schema_path << "' failed to open or is empty.");
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
    const auto json_string = a2d2_to_ros::get_json_file_as_string(json_path);
    if (json_string.empty()) {
      ROS_FATAL_STREAM("'" << json_path << "' failed to open or is empty.");
      return EXIT_FAILURE;
    }

    if (d_json.Parse(json_string.c_str()).HasParseError()) {
      ROS_FATAL_STREAM("Error(offset "
                       << static_cast<unsigned>(d_json.GetErrorOffset())
                       << "): "
                       << rapidjson::GetParseError_En(d_json.GetParseError()));
      return EXIT_FAILURE;
    }
  }

  ROS_INFO_STREAM("Loaded and parsed schema and data set successfully.");

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
    ROS_FATAL_STREAM(ss.str());
    return EXIT_FAILURE;
  }

  ROS_INFO_STREAM("JSON data validated against schema, ready to convert.");

  ///
  /// Ensure that the schema has necessary information for the bag file
  ///

  {
    const auto has_required = d_schema.HasMember("required");
    const auto is_array = d_schema["required"].IsArray();
    const auto is_empty = (is_array && d_schema["required"].GetArray().Empty());
    if (!has_required || !is_array || is_empty) {
      ROS_FATAL_STREAM(
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
        ROS_FATAL_STREAM("Required field at index "
                         << idx << " is not a string type.");
        return EXIT_FAILURE;
      }

      const auto name = std::string(r[idx].GetString());
      if (name.empty()) {
        ROS_FATAL_STREAM("Required field name at index " << idx
                                                         << " is empty.");
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
  bag.open(output_path + "/" + file_basename + ".bag", rosbag::bagmode::Write);
  const rapidjson::Value& r = d_schema["required"];
  for (rapidjson::SizeType idx = 0; idx < r.Size(); ++idx) {
    const auto name = std::string(r[idx].GetString());
    ROS_INFO_STREAM("Converting " << name << "...");

    const rapidjson::Value& obj = d_json[name.c_str()].GetObject();
    const rapidjson::Value& values = obj["values"];
    const auto units = obj["unit"].IsNull() ? "null" : obj["unit"].GetString();

    // if original data is included, publish the units
    if (include_original) {
      const rapidjson::Value& t_v = values[static_cast<rapidjson::SizeType>(0)];
      const auto time = t_v[static_cast<rapidjson::SizeType>(0)].GetUint64();
      const auto first_time = a2d2_to_ros::a2d2_timestamp_to_ros_time(time);

      std_msgs::String units_msg;
      units_msg.data = units;

      bag.write(topic_prefix + "/" + name + "/" + _ORIGINAL_UNITS_TOPIC,
                first_time, units_msg);
    }

    // publish data for each of the values in this field
    for (rapidjson::SizeType idx = 0; idx < values.Size(); ++idx) {
      const rapidjson::Value& t_v = values[idx];
      const auto time = t_v[static_cast<rapidjson::SizeType>(0)].GetUint64();
      if (!a2d2_to_ros::valid_ros_timestamp(time)) {
        ROS_FATAL_STREAM("Timestamp "
                         << time
                         << " has unsupported magnitude: ROS does not support "
                            "timestamps on or after 4294967296000000 "
                            "(Sunday, February 7, 2106 6:28:16 AM GMT)\nCall "
                            "Zager and Evans for details.");
        return EXIT_FAILURE;
      }

      const auto value = t_v[static_cast<rapidjson::SizeType>(1)].GetDouble();
      const auto data =
          a2d2_to_ros::DataPair::build(value, time, bus_frame_name);

      const auto& stamp = data.header.stamp;
      stamps.insert(stamp);
      bag.write(topic_prefix + "/" + name + "/" + _HEADER_TOPC, stamp,
                data.header);
      if (include_original) {
        bag.write(topic_prefix + "/" + name + "/" + _ORIGINAL_VALUE_TOPIC,
                  stamp, data.value);
      }
      if (include_converted) {
        const auto ros_value =
            a2d2_to_ros::to_ros_units(units, data.value.data);
        std_msgs::Float64 ros_value_msg;
        ros_value_msg.data = ros_value;
        bag.write(topic_prefix + "/" + name + "/" + _VALUE_TOPIC, stamp,
                  ros_value_msg);
      }
    }
  }

  ///
  /// Write a clock message for every unique timestamp in the data set
  ///

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
