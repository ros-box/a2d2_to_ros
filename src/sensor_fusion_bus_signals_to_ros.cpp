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
#include <sstream>

#include <ros/console.h>
#include <ros/ros.h>
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/schema.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

// uncomment this define to log warnings and errors
#define _ENABLE_A2D2_ROS_LOGGING_
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"

namespace {
const std::string OUTPUT_PATH =
    "/home/maeve/data/a2d2/Ingolstadt/Bus "
    "Signals/camera_lidar/20190401_145936/bus/20190401145936_bus_signals.bag";
const std::string SCHEMA_PATH =
    "/home/maeve/catkin_ws/src/a2d2_to_ros/schemas/"
    "sensor_fusion_bus_signal.schema";
const std::string JSON_PATH =
    "/home/maeve/data/a2d2/Ingolstadt/Bus "
    "Signals/camera_lidar/20190401_145936/bus/20190401145936_bus_signals.json";
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

  // extract data
  // write to bag
  // done
  return EXIT_SUCCESS;
}
