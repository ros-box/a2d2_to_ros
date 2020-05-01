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
#include "rapidjson/writer.h"

// uncomment this define to log warnings and errors
#define _ENABLE_A2D2_ROS_LOGGING_
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"

typedef std::set<a2d2_to_ros::DataPair, a2d2_to_ros::DataPairTimeComparator>
    DataPairSet;
typedef std::unordered_map<std::string, std::tuple<std::string, DataPairSet>>
    DataPairMap;

namespace {
const std::string CLOCK_TOPIC = "/clock";
const std::string ORIGINAL_VALUE_TOPIC = "original_value";
const std::string ORIGINAL_UNITS_TOPIC = "original_units";
const std::string VALUE_TOPIC = "ros_value";
const std::string HEADER_TOPC = "header";

const std::string OUTPUT_PATH =
    "/home/maeve/data/a2d2/Ingolstadt/Bus "
    "Signals/camera_lidar/20190401_145936/bus/20190401145936_bus_signals.bag";
const std::string SCHEMA_PATH =
    "/home/maeve/catkin_ws/src/a2d2_to_ros/schemas/"
    "sensor_fusion_bus_signal.schema";
const std::string JSON_PATH =
    "/home/maeve/data/a2d2/Ingolstadt/Bus "
    "Signals/camera_lidar/20190401_145936/bus/20190401145936_bus_signals.json";
const std::string FRAME_NAME = "bus";
}  // namespace

int main(int argc, char* argv[]) {
  // parse schema
  rapidjson::Document d_schema;
  {
    // get schema file string
    const auto schema_string =
        a2d2_to_ros::get_json_file_as_string(SCHEMA_PATH);
    if (schema_string.empty()) {
      ROS_FATAL_STREAM("'" << SCHEMA_PATH << "' failed to open or is empty.");
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

  // parse json
  rapidjson::Document d_json;
  {
    // get json file string
    const auto json_string = a2d2_to_ros::get_json_file_as_string(JSON_PATH);
    if (json_string.empty()) {
      ROS_FATAL_STREAM("'" << JSON_PATH << "' failed to open or is empty.");
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

  ROS_INFO_STREAM("Loaded and parsed everything successfully.");

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

  const auto has_required = d_schema.HasMember("required");
  const auto is_array = d_schema["required"].IsArray();
  if (!has_required || !is_array) {
    ROS_FATAL_STREAM(
        "Schema either does not have a 'required' member, or the member is not "
        "an array: HasMember('required'): "
        << has_required << ", isArray(): " << is_array);
    return EXIT_FAILURE;
  }

  DataPairMap data_map;
  const rapidjson::Value& r = d_schema["required"];
  for (rapidjson::SizeType idx = 0; idx < r.Size(); ++idx) {
    if (!r[idx].IsString()) {
      ROS_FATAL_STREAM("Required field at index " << idx
                                                  << " is not a string type.");
      return EXIT_FAILURE;
    }

    const auto field_name = std::string(r[idx].GetString());
    if (field_name.empty()) {
      ROS_FATAL_STREAM("Required field name at index " << idx << " is empty.");
      return EXIT_FAILURE;
    }

    data_map[field_name] = std::make_tuple("", DataPairSet());
  }

  const auto get_units = [](const rapidjson::Value& obj) {
    if (obj["unit"].IsNull()) {
      return std::string("null");
    }
    return std::string(obj["unit"].GetString());
  };

  // get topics (top-level required items in schema)
  for (const auto& pair : data_map) {
    const auto name = pair.first.c_str();
    const rapidjson::Value& obj = d_json[name].GetObject();
    const auto units = get_units(obj);
    const rapidjson::Value& values = obj["values"];

    DataPairSet data_set;
    for (rapidjson::SizeType idx = 0; idx < values.Size(); ++idx) {
      const rapidjson::Value& t_v = values[idx];
      const auto time = t_v[static_cast<rapidjson::SizeType>(0)].GetUint64();
      const auto value = t_v[static_cast<rapidjson::SizeType>(1)].GetDouble();
      data_set.insert(a2d2_to_ros::DataPair::build(value, time, FRAME_NAME));
    }

    data_map[pair.first] = std::make_tuple(units, std::move(data_set));
  }

  // write to bag
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Write);
  for (const auto& pair : data_map) {
    ROS_INFO_STREAM("Converting " << pair.first << " data...");
    const auto& name = pair.first;
    const auto& units = std::get<0>(pair.second);
    const auto& data_set = std::get<1>(pair.second);
    if (data_set.empty()) {
      continue;
    }

    const auto units_enum = a2d2_to_ros::get_unit_enum(units);
    std_msgs::String units_msg;
    units_msg.data = units;
    const auto first_time = data_set.begin()->header.stamp;
    bag.write(name + "/" + ORIGINAL_UNITS_TOPIC, first_time, units_msg);
    for (const auto& data : data_set) {
      rosgraph_msgs::Clock clock_msg;
      clock_msg.clock = data.header.stamp;
      //      bag.write(CLOCK_TOPIC, data.header.stamp, clock_msg); don't do
      //      this; collect all unique timestamps into single set, write them
      //      separately
      bag.write(name + "/" + HEADER_TOPC, data.header.stamp, data.header);
      bag.write(name + "/" + ORIGINAL_VALUE_TOPIC, data.header.stamp,
                data.value);

      const auto ros_value =
          a2d2_to_ros::to_ros_units(units_enum, data.value.data);
      std_msgs::Float64 ros_value_msg;
      ros_value_msg.data = ros_value;
      bag.write(name + "/" + VALUE_TOPIC, data.header.stamp, ros_value_msg);
    }
  }

  bag.close();

  return EXIT_SUCCESS;
}
