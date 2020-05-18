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

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>

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
static constexpr auto _DATASET_SUFFIX = "lidar";
static constexpr auto _INCLUDE_CLOCK_TOPIC = true;
static constexpr auto _INCLUDE_DEPTH_MAP = false;
static constexpr auto _VERBOSE = false;
static constexpr auto _MIN_TIME_OFFSET = 0.0;
static constexpr auto _DURATION = std::numeric_limits<double>::max();

int main(int argc, char* argv[]) {
  BUILD_INFO;  // just write to log what build options were specified

  ///
  /// Set up command line arguments
  ///

  // TODO(jeff): rename "reflectance" to "intensity" assuming that's what it is
  boost::optional<std::string> camera_frame_schema_path_opt;
  boost::optional<std::string> lidar_path_opt;
  boost::optional<std::string> camera_path_opt;
  po::options_description desc(
      "Convert sequential lidar data to rosbag for the A2D2 Sensor Fusion "
      "data set. See README.md for details.\nAvailable options are listed "
      "below. Arguments without default values are required",
      _PROGRAM_OPTIONS_LINE_LENGTH);
  desc.add_options()("help,h", "Print help and exit.")(
      "lidar-data-path,d", po::value(&lidar_path_opt)->required(),
      "Path to the lidar data files.")(
      "camera-data-path,c", po::value(&camera_path_opt)->required(),
      "Path to the camera data files (for timestamp information).")(
      "frame-info-schema-path,s",
      po::value(&camera_frame_schema_path_opt)->required(),
      "Path to the JSON schema for camera frame info files.")(
      "include-clock-topic,t",
      po::value<bool>()->default_value(_INCLUDE_CLOCK_TOPIC),
      "Optional: Use timestamps from the data to write a /clock topic.")(
      "min-time-offset,m", po::value<double>()->default_value(_MIN_TIME_OFFSET),
      "Optional: Seconds to skip ahead in the data before starting the bag.")(
      "duration,d", po::value<double>()->default_value(_DURATION),
      "Optional: Seconds after min-time-offset to include in bag file.")(
      "output-path,o", po::value<std::string>()->default_value(_OUTPUT_PATH),
      "Optional: Path for the output bag file.")(
      "include-depth-map,i",
      po::value<bool>()->default_value(_INCLUDE_DEPTH_MAP),
      "Optional: Publish a depth map version of the lidar data.")(
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

  const auto camera_frame_schema_path = *camera_frame_schema_path_opt;
  const auto camera_path = *camera_path_opt;
  const auto lidar_path = *lidar_path_opt;
  const auto output_path = vm["output-path"].as<std::string>();
  const auto include_depth_map = vm["include-depth-map"].as<bool>();
  const auto include_clock_topic = vm["include-clock-topic"].as<bool>();
  const auto verbose = vm["verbose"].as<bool>();
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

  boost::filesystem::path d(lidar_path);
  const auto timestamp = d.parent_path().parent_path().filename().string();

  const auto file_basename =
      (timestamp + "_" + boost::filesystem::basename(lidar_path));

  ///
  /// Get list of .npz file names
  ///

  std::set<std::string> files;
  boost::filesystem::directory_iterator it{d};
  while (it != boost::filesystem::directory_iterator{}) {
    const auto path = it->path().string();
    const auto extension = it->path().extension().string();
    ++it;
    if (extension != ".npz") {
      continue;
    }
    files.insert(path);
  }

  ///
  /// Get the JSON schema for the camera frame info files
  ///

  rapidjson::Document camera_frame_d;
  {
    // get schema file string
    const auto schema_string =
        a2d2::get_json_file_as_string(camera_frame_schema_path);
    if (schema_string.empty()) {
      X_FATAL("'" << camera_frame_schema_path
                  << "' failed to open or is empty.");
      return EXIT_FAILURE;
    }

    if (camera_frame_d.Parse(schema_string.c_str()).HasParseError()) {
      fprintf(stderr, "\nError(offset %u): %s\n",
              static_cast<unsigned>(camera_frame_d.GetErrorOffset()),
              rapidjson::GetParseError_En(camera_frame_d.GetParseError()));
      return EXIT_FAILURE;
    }
  }
  rapidjson::SchemaDocument camera_frame_schema(camera_frame_d);

  ///
  /// Load each npz file, convert to PointCloud2 message, write to bag
  ///

  const auto fields = a2d2::npz::Fields::get_fields();

  X_INFO("Attempting to convert point cloud data. This may take a while...");

  std::set<ros::Time> stamps;
  rosbag::Bag bag;
  const auto bag_name = output_path + "/" + file_basename + "_" +
                        std::string(_DATASET_SUFFIX) + ".bag";
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
      const auto camera_basename = a2d2::camera_name_from_lidar_name(b);
      if (camera_basename.empty()) {
        X_FATAL("Failed to get camera file corresponding to lidar file: "
                << f << ". Cannot continue.");
        bag.close();
        return EXIT_FAILURE;
      }

      const auto camera_data_file =
          camera_path + "/" + camera_basename + ".json";
      // get json file string
      const auto json_string = a2d2::get_json_file_as_string(camera_data_file);
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

      rapidjson::SchemaValidator validator(camera_frame_schema);
      if (!d_json.Accept(validator)) {
        rapidjson::StringBuffer sb;
        validator.GetInvalidSchemaPointer().StringifyUriFragment(sb);
        std::stringstream ss;
        ss << "\nInvalid schema: " << sb.GetString() << "\n";
        ss << "Invalid keyword: " << validator.GetInvalidSchemaKeyword()
           << "\n";
        sb.Clear();
        validator.GetInvalidDocumentPointer().StringifyUriFragment(sb);
        ss << "Invalid document: " << sb.GetString() << "\n";
        X_FATAL(ss.str());
        bag.close();
        return EXIT_FAILURE;
      } else {
        // X_INFO("Validated: " << camera_data_file);
      }

      const auto frame_timestamp = d_json["cam_tstamp"].GetUint64();
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
    /// Load and verify the data
    ///

    cnpy::npz_t npz;
    try {
      npz = cnpy::npz_load(f);
    } catch (const std::exception& e) {
      X_FATAL(e.what());
      bag.close();
      return EXIT_FAILURE;
    }

    const auto npz_structure_valid = a2d2::verify_npz_structure(npz);
    if (!npz_structure_valid) {
      X_FATAL("Encountered unexpected structure in the data. Cannot continue.");
      bag.close();
      return EXIT_FAILURE;
    } else {
      // X_INFO("Successfully loaded npz data from:\n" << f);
    }

    ///
    /// Build pointcloud message
    ///

    // capture these up front; they provide meta information about the data
    const auto& points = npz[fields[a2d2::npz::Fields::POINTS_IDX]];
    const auto& timestamp = npz[fields[a2d2::npz::Fields::TIMESTAMP_IDX]];
    const auto& valid = npz[fields[a2d2::npz::Fields::VALID_IDX]];

    const auto lidar_file_name = a2d2::frame_from_filename(f);
    const auto lidar_name =
        a2d2::get_camera_name_from_frame_name(lidar_file_name);
    const auto frame =
        a2d2::tf_frame_name(a2d2::sensors::Names::CAMERAS, lidar_name);
    if (frame.empty()) {
      X_FATAL("Could not find frame name in filename: "
              << f << ". Cannot continue.");
      bag.close();
      return EXIT_FAILURE;
    }

    const auto is_dense = a2d2::any_lidar_points_invalid(valid);
    const auto n_points = points.shape[a2d2::npz::Fields::ROW_SHAPE_IDX];
    auto msg = a2d2::build_pc2_msg(frame, frame_timestamp_ros, is_dense,
                                   static_cast<uint32_t>(n_points));

    ///
    /// Fill in the point cloud message
    ///

    auto iters = a2d2::A2D2_PointCloudIterators(msg, fields);
    for (auto row = 0; row < n_points; ++row, ++iters) {
      ///
      /// Point data
      ///

      {
        constexpr auto X_POS = 0;
        constexpr auto Y_POS = 1;
        constexpr auto Z_POS = 2;
        const auto row_step = points.shape[a2d2::npz::Fields::COL_SHAPE_IDX];

        const auto data = points.data<a2d2::npz::ReadTypes::Point>();
        const auto x_idx = a2d2::flatten_2d_index(row_step, row, X_POS);
        const auto y_idx = a2d2::flatten_2d_index(row_step, row, Y_POS);
        const auto z_idx = a2d2::flatten_2d_index(row_step, row, Z_POS);

        *(iters.x) = static_cast<a2d2::npz::WriteTypes::Point>(data[x_idx]);
        *(iters.y) = static_cast<a2d2::npz::WriteTypes::Point>(data[y_idx]);
        *(iters.z) = static_cast<a2d2::npz::WriteTypes::Point>(data[z_idx]);
      }

      ///
      /// Scalar data
      ///

      {
        const auto& azimuth = npz[fields[a2d2::npz::Fields::AZIMUTH_IDX]];
        const auto data = azimuth.data<a2d2::npz::ReadTypes::Azimuth>();
        *(iters.azimuth) = data[row];
      }

      {
        const auto& boundary = npz[fields[a2d2::npz::Fields::BOUNDARY_IDX]];
        const auto data = boundary.data<a2d2::npz::ReadTypes::Boundary>();
        *(iters.boundary) =
            static_cast<a2d2::npz::WriteTypes::Boundary>(data[row]);
      }

      {
        const auto& image_col = npz[fields[a2d2::npz::Fields::COL_IDX]];
        const auto data = image_col.data<a2d2::npz::ReadTypes::Col>();
        *(iters.col) = static_cast<a2d2::npz::WriteTypes::Col>(data[row]);
      }

      {
        const auto& depth = npz[fields[a2d2::npz::Fields::DEPTH_IDX]];
        const auto data = depth.data<a2d2::npz::ReadTypes::Depth>();
        *(iters.depth) = static_cast<a2d2::npz::WriteTypes::Depth>(data[row]);
      }

      {
        const auto& distance = npz[fields[a2d2::npz::Fields::DISTANCE_IDX]];
        const auto data = distance.data<a2d2::npz::ReadTypes::Distance>();
        *(iters.distance) =
            static_cast<a2d2::npz::WriteTypes::Distance>(data[row]);
      }

      {
        const auto& lidar_id = npz[fields[a2d2::npz::Fields::ID_IDX]];
        const auto data = lidar_id.data<a2d2::npz::ReadTypes::LidarId>();
        *(iters.lidar_id) =
            static_cast<a2d2::npz::WriteTypes::LidarId>(data[row]);
      }

      {
        const auto& rectime = npz[fields[a2d2::npz::Fields::RECTIME_IDX]];
        const auto data = rectime.data<a2d2::npz::ReadTypes::Rectime>();
        *(iters.rectime) =
            static_cast<a2d2::npz::WriteTypes::Rectime>(data[row]);
      }

      {
        const auto& reflectance =
            npz[fields[a2d2::npz::Fields::REFLECTANCE_IDX]];
        const auto data = reflectance.data<a2d2::npz::ReadTypes::Reflectance>();
        *(iters.reflectance) =
            static_cast<a2d2::npz::WriteTypes::Reflectance>(data[row]);
      }

      {
        const auto& image_row = npz[fields[a2d2::npz::Fields::ROW_IDX]];
        const auto data = image_row.data<a2d2::npz::ReadTypes::Row>();
        *(iters.row) = static_cast<a2d2::npz::WriteTypes::Row>(data[row]);
      }

      {
        const auto data = timestamp.data<a2d2::npz::ReadTypes::Timestamp>();
        *(iters.timestamp) =
            static_cast<a2d2::npz::WriteTypes::Timestamp>(data[row]);
      }

      {
        const auto data = valid.data<a2d2::npz::ReadTypes::Valid>();
        *(iters.valid) = static_cast<a2d2::npz::WriteTypes::Valid>(data[row]);
      }
    }

#if 0
    {
      const auto& reflectance = npz[fields[a2d2::npz::Fields::REFLECTANCE_IDX]];
      const auto max_reflectance =
          a2d2::get_max_value<a2d2::npz::ReadTypes::Reflectance>(reflectance);
      const auto min_reflectance =
          a2d2::get_min_value<a2d2::npz::ReadTypes::Reflectance>(reflectance);
      std::cout << "reflectance range: [" << min_reflectance << ", "
                << max_reflectance << "]" << std::endl;
      const auto& boundary = npz[fields[a2d2::npz::Fields::BOUNDARY_IDX]];
      const auto max_boundary =
          a2d2::get_max_value<a2d2::npz::ReadTypes::Boundary>(boundary);
      const auto min_boundary =
          a2d2::get_min_value<a2d2::npz::ReadTypes::Boundary>(boundary);
      std::cout << "boundary range: [" << min_boundary << ", " << max_boundary
                << "]" << std::endl;
      auto test = a2d2::A2D2_PointCloudIterators(msg, fields);
      for (auto row = 0; row < n_points; ++row, ++test) {
        std::cout << test << std::endl;
      }
      X_WARN("Finished debut output. Exiting.");
      bag.close();
      return EXIT_FAILURE;
    }
#endif

    ///
    /// Write message to bag file
    ///

    // message time is the max timestamp of all points in the message
    const auto topic = (std::string(_DATASET_NAMESPACE) + "/" + file_basename +
                        "/" + std::string(_DATASET_SUFFIX));
    bag.write(topic, msg.header.stamp, msg);
    if (include_clock_topic) {
      stamps.insert(msg.header.stamp);
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
