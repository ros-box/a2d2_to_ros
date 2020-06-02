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

#include <boost/filesystem/convenience.hpp>  // TODO(jeff): use std::filesystem in C++17
#include <boost/optional.hpp>
#include <boost/program_options.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "a2d2_to_ros/json_utils.hpp"
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"
#include "a2d2_to_ros/log_build_options.hpp"
#include "a2d2_to_ros/logging.hpp"

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
static constexpr auto _DATASET_SUFFIX = "camera";
static constexpr auto _VERBOSE = false;
static constexpr auto _INCLUDE_CLOCK_TOPIC = false;
static constexpr auto _START_TIME = static_cast<uint64_t>(0);
static constexpr auto _MIN_TIME_OFFSET = 0.0;
static constexpr auto _DURATION = std::numeric_limits<double>::max();

int main(int argc, char* argv[]) {
  X_INFO("<Camera Converter>");
  BUILD_INFO;  // just write to log what build options were specified

  ///
  /// Set up command line arguments
  ///

  // TODO(jeff): rename "reflectance" to "intensity" assuming that's what it is
  boost::optional<std::string> camera_path_opt;
  boost::optional<std::string> camera_frame_schema_path_opt;
  boost::optional<std::string> sensor_config_path_opt;
  boost::optional<std::string> sensor_config_schema_path_opt;
  po::options_description desc(
      "Convert sequential camera data to rosbag for the A2D2 Sensor Fusion "
      "data set. See README.md for details.\nAvailable options are listed "
      "below. Arguments without default values are required",
      _PROGRAM_OPTIONS_LINE_LENGTH);
  desc.add_options()("help,h", "Print help and exit.")(
      "camera-data-path,c", po::value(&camera_path_opt)->required(),
      "Path to the camera data files.")(
      "frame-info-schema-path,s",
      po::value(&camera_frame_schema_path_opt)->required(),
      "Path to the JSON schema for camera frame info files.")(
      "sensor-config-path,c", po::value(&sensor_config_path_opt)->required(),
      "Path to the JSON for vehicle/sensor config.")(
      "sensor-config-schema-path,s",
      po::value(&sensor_config_schema_path_opt)->required(),
      "Path to the JSON schema for the vehicle/sensor config.")(
      "include-clock-topic,t",
      po::value<bool>()->default_value(_INCLUDE_CLOCK_TOPIC),
      "Optional: Use timestamps from the data to write a /clock topic.")(
      "start-time,a", po::value<uint64_t>()->default_value(_START_TIME),
      "Optional: Only convert data recorded at or after this time (TAI "
      "microseconds).")(
      "min-time-offset,m", po::value<double>()->default_value(_MIN_TIME_OFFSET),
      "Optional: Seconds to skip ahead in the data before starting the bag.")(
      "duration,d", po::value<double>()->default_value(_DURATION),
      "Optional: Seconds after min-time-offset to include in bag file.")(
      "output-path,o", po::value<std::string>()->default_value(_OUTPUT_PATH),
      "Optional: Path for the output bag file.")(
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

  const auto camera_path = *camera_path_opt;
  const auto camera_frame_schema_path = *camera_frame_schema_path_opt;
  const auto sensor_config_path = *sensor_config_path_opt + "/cams_lidars.json";
  const auto sensor_config_schema_path = *sensor_config_schema_path_opt;
  const auto output_path = vm["output-path"].as<std::string>();
  const auto verbose = vm["verbose"].as<bool>();
  const auto include_clock_topic = vm["include-clock-topic"].as<bool>();
  const auto start_time = vm["start-time"].as<uint64_t>();
  const auto min_time_offset = vm["min-time-offset"].as<double>();
  const auto duration = vm["duration"].as<double>();

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

  auto d_schema_opt = a2d2::get_rapidjson_dom(sensor_config_schema_path);
  if (!d_schema_opt) {
    X_FATAL("Could not open '" << sensor_config_schema_path);
    return EXIT_FAILURE;
  }
  auto& schema_d = *d_schema_opt;
  rapidjson::SchemaDocument config_schema(schema_d);

  ///
  /// Validate the data set against the schema
  ///

  {
    rapidjson::SchemaValidator validator(config_schema);
    if (!sensor_config_d.Accept(validator)) {
      const auto err_string = a2d2::get_validator_error_string(validator);
      X_FATAL(err_string);
      return EXIT_FAILURE;
    }
    X_INFO("Validated: " << sensor_config_schema_path);
  }

  ///
  /// Generate camera info messages
  ///
  std::unordered_map<std::string, sensor_msgs::CameraInfo> camera_info_msgs;
  {
    const auto sensor_names = a2d2::sensors::Frames::get_sensors();
    for (const auto& name : sensor_names) {
      // No cameras at these positions
      const auto is_rear_left = (name == "rear_left");
      const auto is_rear_right = (name == "rear_right");
      if (is_rear_left || is_rear_right) {
        continue;
      }
      if (verbose) {
        X_INFO("Getting camera info for: " << name);
      }

      camera_info_msgs[name] = sensor_msgs::CameraInfo();
      auto& msg = camera_info_msgs[name];

      const rapidjson::Value& camera =
          sensor_config_d[a2d2::sensors::Names::CAMERAS.c_str()][name.c_str()];

      const std::string camera_type = camera["Lens"].GetString();
      const auto is_fisheye = (camera_type == "Fisheye");

      // TODO(jeff): verify that this is right
      msg.D.resize(5, 0.0);
      const auto ROW_IDX = static_cast<rapidjson::SizeType>(0);
      const auto D_size = static_cast<rapidjson::SizeType>(is_fisheye ? 4 : 5);
      for (auto i = 0; i < D_size; ++i) {
        const auto IDX = static_cast<rapidjson::SizeType>(i);
        msg.D[i] = camera["Distortion"][ROW_IDX][IDX].GetDouble();
      }

      {
        const rapidjson::Value& camera_matrix_raw = camera["CamMatrixOriginal"];
        for (auto i = 0; i < camera_matrix_raw.Size(); ++i) {
          const auto row_idx = static_cast<rapidjson::SizeType>(i);
          for (auto j = 0; j < camera_matrix_raw[row_idx].Size(); ++j) {
            const auto col_idx = static_cast<rapidjson::SizeType>(j);
            const auto msg_idx = ((row_idx * 3) + col_idx);
            msg.K[msg_idx] = camera_matrix_raw[row_idx][col_idx].GetDouble();
          }
        }
      }

      {
        msg.P.fill(0.0);
        const rapidjson::Value& camera_matrix = camera["CamMatrix"];
        for (auto i = 0; i < camera_matrix.Size(); ++i) {
          const auto row_idx = static_cast<rapidjson::SizeType>(i);
          for (auto j = 0; j < camera_matrix[row_idx].Size(); ++j) {
            const auto col_idx = static_cast<rapidjson::SizeType>(j);
            const auto msg_idx = ((row_idx * 4) + col_idx);
            msg.P[msg_idx] = camera_matrix[row_idx][col_idx].GetDouble();
          }
        }
      }

      constexpr auto WIDTH_IDX = static_cast<rapidjson::SizeType>(0);
      constexpr auto HEIGHT_IDX = static_cast<rapidjson::SizeType>(1);
      msg.width = camera["Resolution"][WIDTH_IDX].GetInt64();
      msg.height = camera["Resolution"][HEIGHT_IDX].GetInt64();

      msg.binning_x = 0;
      msg.binning_y = 0;

      msg.roi.x_offset = 0;
      msg.roi.y_offset = 0;
      msg.roi.height = 0;
      msg.roi.width = 0;
      msg.roi.do_rectify = false;
    }
  }

  boost::filesystem::path d(camera_path);
  const auto timestamp = d.parent_path().parent_path().filename().string();

  const auto file_basename =
      (timestamp + "_" + boost::filesystem::basename(camera_path));

  ///
  /// Get list of .png file names
  ///

  std::set<std::string> files;
  boost::filesystem::directory_iterator it{d};
  while (it != boost::filesystem::directory_iterator{}) {
    const auto path = it->path().string();
    const auto extension = it->path().extension().string();
    ++it;
    if (extension != ".png") {
      continue;
    }
    files.insert(path);
  }

  ///
  /// Get the JSON schema for the camera frame info files
  ///

  auto d_camera_frame_schema_opt =
      a2d2::get_rapidjson_dom(camera_frame_schema_path);
  if (!d_camera_frame_schema_opt) {
    X_FATAL("Could not open '" << camera_frame_schema_path);
    return EXIT_FAILURE;
  }
  auto& camera_frame_d = *d_camera_frame_schema_opt;
  rapidjson::SchemaDocument camera_frame_schema(camera_frame_d);

  ///
  /// Load each png file, convert to Image message, write to bag
  ///

  X_INFO("Attempting to convert camera data. This may take a while...");

  std::set<ros::Time> stamps;
  rosbag::Bag bag;
  const auto bag_name = output_path + "/" + file_basename + "_" +
                        std::string(_DATASET_SUFFIX) + ".bag";
  X_INFO("Creating bag file at: " << bag_name);
  bag.open(bag_name, rosbag::bagmode::Write);
  boost::optional<ros::Time> first_time;
  for (const auto& f : files) {
    ///
    /// Get camera data file for timestamp information
    ///

    ros::Time frame_timestamp_ros;
    {
      const auto p = boost::filesystem::path(f);
      const auto b = boost::filesystem::basename(p);
      const auto camera_data_file = (camera_path + "/" + b + ".json");
      const auto json_string = a2d2::get_file_as_string(camera_data_file);
      if (json_string.empty()) {
        X_FATAL("'" << camera_data_file << "' failed to open or is empty.");
        bag.close();
        return EXIT_FAILURE;
      }

      rapidjson::Document d_json;
      if (d_json.Parse(json_string.c_str()).HasParseError()) {
        X_FATAL("Error(offset "
                << static_cast<unsigned>(d_json.GetErrorOffset()) << "): "
                << rapidjson::GetParseError_En(d_json.GetParseError()));
        bag.close();
        return EXIT_FAILURE;
      }

      ///
      /// Validate the data set against the schema
      ///

      {
        rapidjson::SchemaValidator validator(camera_frame_schema);
        if (!d_json.Accept(validator)) {
          const auto err_string = a2d2::get_validator_error_string(validator);
          X_FATAL(err_string);
          bag.close();
          return EXIT_FAILURE;
        }
        if (verbose) {
          X_INFO("Validated: " << camera_data_file);
        }
      }

      const auto frame_timestamp = d_json["cam_tstamp"].GetUint64();

      if (frame_timestamp < start_time) {
        continue;
      }

      frame_timestamp_ros = a2d2::a2d2_timestamp_to_ros_time(frame_timestamp);
    }

    if (!first_time) {
      first_time = frame_timestamp_ros;
    }

    const auto time_since_begin = (frame_timestamp_ros - *first_time).toSec();
    if (time_since_begin < min_time_offset) {
      continue;
    }

    const auto recorded_duration = (time_since_begin - min_time_offset);
    if (recorded_duration > duration) {
      break;
    }

    ///
    /// Build image message
    ///

    const auto camera_file_name = a2d2::frame_from_filename(f);
    const auto camera_name =
        a2d2::get_camera_name_from_frame_name(camera_file_name);
    const auto frame =
        a2d2::tf_frame_name(a2d2::sensors::Names::CAMERAS, camera_name);
    if (frame.empty()) {
      X_FATAL("Could not find frame name in filename: "
              << f << ". Cannot continue.");
      bag.close();
      return EXIT_FAILURE;
    }

    std_msgs::Header header;
    header.frame_id = frame;
    header.stamp = frame_timestamp_ros;

    cv::Mat img = cv::imread(f);
    auto msg_ptr = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();

    auto it_cam_info = camera_info_msgs.find(camera_name);
    if (std::end(camera_info_msgs) == it_cam_info) {
      X_ERROR("Did not find camera info for: " << camera_name);
    } else {
      // X_INFO("Found camera info for: " << camera_name);
    }
    it_cam_info->second.header = msg_ptr->header;

    ///
    /// Write message to bag file
    ///

    // message time is the max timestamp of all points in the message
    const auto image_topic =
        (std::string(_DATASET_NAMESPACE) + "/" + file_basename + "/" +
         std::string(_DATASET_SUFFIX));
    const auto info_topic = (std::string(_DATASET_NAMESPACE) + "/" +
                             file_basename + "/camera_info");
    bag.write(image_topic, msg_ptr->header.stamp, *msg_ptr);
    bag.write(info_topic, msg_ptr->header.stamp, it_cam_info->second);

    if (include_clock_topic) {
      stamps.insert(msg_ptr->header.stamp);
    }

    if (verbose) {
      X_INFO("Processed: " << f);
    }
  }

  ///
  /// Write a clock message for every unique timestamp in the data set
  ///

  if (include_clock_topic) {
    X_INFO("Adding " << _CLOCK_TOPIC << " topic...");
    if (stamps.size() != files.size()) {
      X_WARN("Number of frame timestamps ("
             << stamps.size()
             << ") is different than the total number of frames ("
             << files.size()
             << "). This should only happen if the min time offset and/or "
                "duration excludes some parts of the data set.");
    }
  }

  for (const auto& stamp : stamps) {
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = stamp;
    bag.write(_CLOCK_TOPIC, stamp, clock_msg);
  }

  bag.close();

  X_INFO("Done.");
  return EXIT_SUCCESS;
}
