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
#include <limits>
#include <set>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <boost/filesystem/convenience.hpp>  // TODO(jeff): use std::filesystem in C++17
#include <boost/optional.hpp>
#include <boost/program_options.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_msgs/TFMessage.h>
#ifdef USE_FLOAT64
#include <std_msgs/Float64.h>
#else
#include <std_msgs/Float32.h>
#endif
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include "a2d2_to_ros/json_utils.hpp"
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"
#include "a2d2_to_ros/log_build_options.hpp"
#include "a2d2_to_ros/logging.hpp"
#include "ros_cnpy/cnpy.h"

///
/// Program constants and defaults.
///

static constexpr auto EPS = 1e-8;
static constexpr auto _TF_FREQUENCEY = 10.0;
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
static constexpr auto _START_TIME = static_cast<uint64_t>(0);
static constexpr auto _MIN_TIME_OFFSET = 0.0;
static constexpr auto _VERBOSE = false;
static constexpr auto _DURATION = std::numeric_limits<double>::max();

///
/// Executable specific stuff
///

#define VERIFY_BASIS_ORIGIN(basis, origin, sensors, frame)                  \
  {                                                                         \
    if (!a2d2::vector_is_valid(origin)) {                                   \
      X_FATAL("Origin for " << sensors << "::" << frame                     \
                            << " is not valid. Origin must be finite and "  \
                               "real valued. Cannot continue.");            \
      return EXIT_FAILURE;                                                  \
    }                                                                       \
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
}  // namespace

typedef std::set<a2d2::DataPair, a2d2::DataPairTimeComparator> DataPairSet;
typedef std::unordered_map<std::string, std::tuple<std::string, DataPairSet>>
    DataPairMap;

int main(int argc, char* argv[]) {
  X_INFO("<Bus Signal Converter>");
  BUILD_INFO;  // just write to log what build options were specified

  ///
  /// Set up command line arguments
  ///

  boost::optional<std::string> schema_path_opt;
  boost::optional<std::string> json_path_opt;
  boost::optional<std::string> sensor_config_path_opt;
  boost::optional<std::string> sensor_config_schema_path_opt;

  po::options_description desc(
      "Convert sequential bus signal data to rosbag for the A2D2 Sensor Fusion "
      "data set. In addition, write a transform bag file containing the "
      "vehicle box model and tf tree for the vehicle sensor configuration. See "
      "README.md for details.\nAvailable options are listed  below. Arguments "
      "without default values are required",
      _PROGRAM_OPTIONS_LINE_LENGTH);
  desc.add_options()("help,h", "Print help and exit.")(
      "sensor-config-json-path,c",
      po::value(&sensor_config_path_opt)->required(),
      "Path to the directory containing the JSON for vehicle/sensor config.")(
      "sensor-config-schema-path,s",
      po::value(&sensor_config_schema_path_opt)->required(),
      "Path to the JSON schema for the vehicle/sensor config.")(
      "bus-signal-json-path,j", po::value(&json_path_opt)->required(),
      "Path to the directory containing the JSON bus signal file.")(
      "bus-signal-schema-path,b", po::value(&schema_path_opt)->required(),
      "Path to the JSON schema for bus signal data.")(
      "start-time,a", po::value<uint64_t>()->default_value(_START_TIME),
      "Optional: Start on or after this time (TAI microseconds).")(
      "min-time-offset,m", po::value<double>()->default_value(_MIN_TIME_OFFSET),
      "Optional: Seconds to skip ahead in the data before starting the bag.")(
      "duration,d", po::value<double>()->default_value(_DURATION),
      "Optional: Seconds after min-time-offset to include in bag file.")(
      "output-path,o", po::value<std::string>()->default_value(_OUTPUT_PATH),
      "Optional: Path for the output bag file.")(
      "include-clock-topic,t",
      po::value<bool>()->default_value(_INCLUDE_CLOCK_TOPIC),
      "Optional: Write bus signal times to a /clock topic in the TF bag.")(
      "include-original-values,i",
      po::value<bool>()->default_value(_INCLUDE_ORIGINAL),
      "Optional: Include data set values in their original units.")(
      "include-converted-values,r",
      po::value<bool>()->default_value(_INCLUDE_CONVERTED),
      "Optional: Include data set values converted to ROS standard units.")(
      "verbose,v", po::value<bool>()->default_value(_VERBOSE),
      "Optional: Show name of each file after it is processed.");

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

  const auto sensor_config_path = *sensor_config_path_opt + "/cams_lidars.json";
  const auto sensor_config_schema_path = *sensor_config_schema_path_opt;
  const auto schema_path = *schema_path_opt;
  const auto json_data_path = *json_path_opt;
  const auto output_path = vm["output-path"].as<std::string>();
  const auto include_original = vm["include-original-values"].as<bool>();
  const auto include_converted = vm["include-converted-values"].as<bool>();
  const auto include_clock_topic = vm["include-clock-topic"].as<bool>();
  const auto start_time = vm["start-time"].as<uint64_t>();
  const auto min_time_offset = vm["min-time-offset"].as<double>();
  const auto duration = vm["duration"].as<double>();
  const auto verbose = vm["verbose"].as<bool>();

  const auto valid_min_offset = (std::isfinite(min_time_offset) &&

                                 a2d2::strictly_non_negative(min_time_offset));
  const auto valid_duration =
      (std::isfinite(duration) && a2d2::strictly_non_negative(duration));
  if (!valid_min_offset || !valid_duration) {
    X_FATAL(
        "Time constraints {min-time-offset: "
        << min_time_offset << ", duration: " << duration
        << "} are not valid. They must be finite, real valued, and >= 0.0.");
    return EXIT_FAILURE;
  }

  ///
  /// Get the path for the bus signal JSON data
  /// There should be only one file in the directory, and it should be the data
  ///
  boost::filesystem::directory_iterator it{json_data_path};
  std::string json_path;
  auto iteration = 0;
  constexpr auto MAX_ITERATIONS = 100;
  while (it != boost::filesystem::directory_iterator{}) {
    const auto path = it->path().string();
    const auto extension = it->path().extension().string();
    if (extension == ".json") {
      json_path = path;
    }
    ++it;
    ++iteration;
    if (iteration >= MAX_ITERATIONS) {
      break;
    }
  }

  if (json_path.empty()) {
    X_FATAL(
        "Could not find bus signal data. Either no json file exists in "
        "location, or the directory contains too many files (examined: "
        << iteration << ", max allowed: " << MAX_ITERATIONS << ").");
    return EXIT_FAILURE;
  }
  X_INFO("Found json file: " << json_path
                             << ", assuming this is bus signal data.");

  const auto file_basename = boost::filesystem::basename(json_path);
  const auto topic_prefix =
      (std::string(_DATASET_NAMESPACE) + "/" + file_basename);

  ///
  /// Get the JSON schema for the data set
  ///

  auto d_schema_opt = a2d2::get_rapidjson_dom(schema_path);
  if (!d_schema_opt) {
    X_FATAL("Could not open '" << schema_path);
    return EXIT_FAILURE;
  }
  auto& d_schema = *d_schema_opt;
  rapidjson::SchemaDocument schema(d_schema);

  ///
  /// Get the JSON data set
  ///

  auto d_json_opt = a2d2::get_rapidjson_dom(json_path);
  if (!d_json_opt) {
    X_FATAL("Could not open '" << json_path);
    return EXIT_FAILURE;
  }
  auto& d_json = *d_json_opt;

  X_INFO("Loaded and parsed schema and data set successfully.");

  ///
  /// Validate the data set against the schema
  ///

  {
    rapidjson::SchemaValidator validator(schema);
    if (!d_json.Accept(validator)) {
      const auto err_string = a2d2::get_validator_error_string(validator);
      X_FATAL(err_string);
      return EXIT_FAILURE;
    }
    X_INFO("Validated: " << json_path);
  }

  ///
  /// Get the JSON for vehicle/sensor config
  ///

  auto d_sensor_config_opt = a2d2::get_rapidjson_dom(sensor_config_path);
  if (!d_sensor_config_opt) {
    X_FATAL("Could not open '" << sensor_config_path);
    return EXIT_FAILURE;
  }
  auto& sensor_config_d = *d_sensor_config_opt;

  ///
  /// Get the JSON schema for the config JSON
  ///

  auto d_sensor_config_schema_opt =
      a2d2::get_rapidjson_dom(sensor_config_schema_path);
  if (!d_sensor_config_schema_opt) {
    X_FATAL("Could not open '" << sensor_config_schema_path);
    return EXIT_FAILURE;
  }
  rapidjson::SchemaDocument config_schema(*d_sensor_config_schema_opt);

  ///
  /// Validate the sensor config against the schema
  ///

  {
    rapidjson::SchemaValidator validator(config_schema);
    if (!sensor_config_d.Accept(validator)) {
      const auto err_string = a2d2::get_validator_error_string(validator);
      X_FATAL(err_string);
      return EXIT_FAILURE;
    }
    X_INFO("Validated: " << sensor_config_path);
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
  /// Get sensor poses
  ///

  const auto sensors = a2d2::sensors::Frames::get_sensors();

  // each block will add its transform message to this container
  tf2_msgs::TFMessage msgtf;

  // For each sensor type...
  for (const auto& name :
       {a2d2::sensors::Names::CAMERAS, a2d2::sensors::Names::LIDARS}) {
    const auto is_camera = (name == a2d2::sensors::Names::CAMERAS);
    const auto is_lidar = (name == a2d2::sensors::Names::LIDARS);

    // For each sensor position...
    for (auto i = 0; i < sensors.size(); ++i) {
      const auto& frame = sensors[i];

      // No lidars at these positions
      const auto is_side_left = (a2d2::sensors::Frames::SIDE_LEFT_IDX == i);
      const auto is_side_right = (a2d2::sensors::Frames::SIDE_RIGHT_IDX == i);
      const auto is_rear_center = (a2d2::sensors::Frames::REAR_CENTER_IDX == i);
      if (is_lidar) {
        if (is_side_left || is_side_right || is_rear_center) {
          continue;
        }
      }

      // No cameras at these positions
      const auto is_rear_left = (a2d2::sensors::Frames::REAR_LEFT_IDX == i);
      const auto is_rear_right = (a2d2::sensors::Frames::REAR_RIGHT_IDX == i);
      if (is_camera) {
        if (is_rear_left || is_rear_right) {
          continue;
        }
      }

      // compute transform between sensor and vehicle
      const Eigen::Matrix3d basis =
          a2d2::json_axes_to_eigen_basis(sensor_config_d, name, frame, EPS);
      const Eigen::Vector3d origin =
          a2d2::json_origin_to_eigen_vector(sensor_config_d, name, frame);
      VERIFY_BASIS_ORIGIN(basis, origin, name, frame);

      const Eigen::Affine3d Tx = a2d2::Tx_global_sensor(basis, origin);

      {
        geometry_msgs::Transform Tx_msg;
        tf::transformEigenToMsg(Tx, Tx_msg);

        geometry_msgs::TransformStamped Tx_stamped_msg;
        Tx_stamped_msg.transform = Tx_msg;
        Tx_stamped_msg.header.frame_id = "chassis";
        Tx_stamped_msg.child_frame_id = a2d2::tf_frame_name(name, frame);
        msgtf.transforms.push_back(Tx_stamped_msg);
      }

      {  // TODO(jeff): Compute this from roll/pitch
        geometry_msgs::Transform Tx_msg;
        tf::transformEigenToMsg(Eigen::Affine3d::Identity(), Tx_msg);

        geometry_msgs::TransformStamped Tx_stamped_msg;
        Tx_stamped_msg.transform = Tx_msg;
        Tx_stamped_msg.header.frame_id = "wheels";
        Tx_stamped_msg.child_frame_id = "chassis";
        msgtf.transforms.push_back(Tx_stamped_msg);
      }
    }
  }

  ///
  /// Get the field names from schema, retrieve data from DOM, write to bag.
  /// This loop does not do error checking on the DOM because it should already
  /// have been validated by the schema.
  ///

  std::map<uint64_t, float> roll_angles;
  std::map<uint64_t, float> pitch_angles;

  std::set<ros::Time> stamps;
  rosbag::Bag bus_signal_bag;
  {
    const auto bag_name = output_path + "/" + file_basename + ".bag";
    bus_signal_bag.open(bag_name, rosbag::bagmode::Write);
  }
  const rapidjson::Value& r = d_schema["required"];
  for (rapidjson::SizeType idx = 0; idx < r.Size(); ++idx) {
    const auto name = std::string(r[idx].GetString());
    if (verbose) {
      X_INFO("Converting " << name << "...");
    }

    const rapidjson::Value& obj = d_json[name.c_str()].GetObject();
    const rapidjson::Value& values = obj["values"];
    const auto units = obj["unit"].IsNull() ? "null" : obj["unit"].GetString();

    // publish data for each of the values in this field
    auto no_units_yet = true;
    boost::optional<ros::Time> first_time;
    for (rapidjson::SizeType idx = 0; idx < values.Size(); ++idx) {
      constexpr auto TIMESTAMP_IDX = static_cast<rapidjson::SizeType>(0);
      constexpr auto VALUE_IDX = static_cast<rapidjson::SizeType>(1);
      const rapidjson::Value& t_v = values[idx];
      const auto time = t_v[TIMESTAMP_IDX].GetUint64();
      if (!a2d2::valid_ros_timestamp(time)) {
        X_FATAL("Timestamp "
                << time
                << " has unsupported magnitude: ROS does not support "
                   "timestamps on or after 4294967296000000 "
                   "(Sunday, February 7, 2106 6:28:16 AM GMT)\nCall "
                   "Zager and Evans for details.");
        return EXIT_FAILURE;
      }

      if (time < start_time) {
        continue;
      }

      const auto value = t_v[VALUE_IDX].GetDouble();
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

      if (name == "roll_angle") {
        if (roll_angles.find(time) != std::end(roll_angles)) {
          X_FATAL("Non unique values for roll angle time: "
                  << time << ". Cannot continue.");
          return EXIT_FAILURE;
        }
        roll_angles[time] = value;
      }

      if (name == "pitch_angle") {
        if (pitch_angles.find(time) != std::end(pitch_angles)) {
          X_FATAL("Non unique values for pitch angle time: "
                  << time << ". Cannot continue.");
          return EXIT_FAILURE;
        }
        pitch_angles[time] = value;
      }

      // TODO(jeff): typo _HEADER_TOPC
      bus_signal_bag.write(topic_prefix + "/" + name + "/" + _HEADER_TOPC,
                           stamp, data.header);
      if (include_original) {
        bus_signal_bag.write(
            topic_prefix + "/" + name + "/" + _ORIGINAL_VALUE_TOPIC, stamp,
            data.value);

        if (no_units_yet) {
          std_msgs::String units_msg;
          units_msg.data = units;

          bus_signal_bag.write(
              topic_prefix + "/" + name + "/" + _ORIGINAL_UNITS_TOPIC, stamp,
              units_msg);
          no_units_yet = false;
        }
      }

      if (include_converted) {
        const auto ros_value = a2d2::to_ros_units(units, data.value.data);
        a2d2::DataPair::value_type ros_value_msg;
        ros_value_msg.data = ros_value;
        bus_signal_bag.write(topic_prefix + "/" + name + "/" + _VALUE_TOPIC,
                             stamp, ros_value_msg);
      }

      if (include_clock_topic) {
        stamps.insert(stamp);
      }
    }
  }

  ///
  /// Finish the bus signal bag
  ///

  bus_signal_bag.close();

  ///
  /// Write TF bag
  ///

  // TODO(jeff)
  if (roll_angles.size() != pitch_angles.size()) {
    X_FATAL("roll/pitch angle size mismatch: "
            << roll_angles.size() << " roll values, but " << pitch_angles.size()
            << " pitch values. Cannot continue.");
    return EXIT_FAILURE;
  }

  ///
  /// Write all tf messages to the bag
  ///

  X_INFO("Writing TF bag file...");

  rosbag::Bag tf_bag;
  {
    const auto bag_name = output_path + "/" + file_basename + "_tf.bag";
    tf_bag.open(bag_name, rosbag::bagmode::Write);
  }
  for (const auto& p : roll_angles) {
    const auto it_pitch = pitch_angles.find(p.first);
    if (it_pitch == std::end(pitch_angles)) {
      X_FATAL("Could not find timestamp "
              << p.first << " from roll data in pitch data. Cannot continue.");
      tf_bag.close();
      return EXIT_FAILURE;
    }

    const auto ros_time = a2d2::a2d2_timestamp_to_ros_time(p.first);
    for (auto& msg : msgtf.transforms) {
      msg.header.stamp = ros_time;
    }
    tf_bag.write("/tf", ros_time, msgtf);
    tf_bag.write("/a2d2/ego_shape", ros_time, ego_shape_msg);
  }

  ///
  /// Write a clock message for every unique timestamp in the data set
  ///

  // TODO(jeff): add this to tf bag
  if (include_clock_topic) {
    X_INFO("Adding " << _CLOCK_TOPIC << " topic to TF bag file...");
  }
  for (const auto& stamp : stamps) {
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = stamp;
    tf_bag.write(_CLOCK_TOPIC, stamp, clock_msg);
  }

  tf_bag.close();
  return EXIT_SUCCESS;
}
