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
#include <set>

#include <boost/filesystem/convenience.hpp>  // TODO(jeff): use std::filesystem in C++17
#include <boost/optional.hpp>
#include <boost/program_options.hpp>

#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud.h>

// uncomment this define to log warnings and errors
#define _ENABLE_A2D2_ROS_LOGGING_
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"
#include "ros_cnpy/cnpy.h"

///
/// Program constants and defaults.
///

static constexpr auto _PROGRAM_OPTIONS_LINE_LENGTH = 120u;
static constexpr auto _CLOCK_TOPIC = "/clock";
static constexpr auto _OUTPUT_PATH = ".";
static constexpr auto _DATASET_NAMESPACE = "/a2d2";
static constexpr auto _INCLUDE_DEPTH_MAP = false;

namespace {
namespace po = boost::program_options;
}  // namespace

int main(int argc, char* argv[]) {
  ///
  /// Set up command line arguments
  ///

  boost::optional<std::string> data_path_opt;
  po::options_description desc(
      "Convert sequential lidar data to rosbag for the A2D2 Sensor Fusion "
      "data set. See README.md for details.\nAvailable options are listed "
      "below. Arguments without default values are required",
      _PROGRAM_OPTIONS_LINE_LENGTH);
  desc.add_options()("help,h", "Print help and exit.")(
      "data-path,d", po::value(&data_path_opt)->required(),
      "Path to the lidar data files.")(
      "output-path,o", po::value<std::string>()->default_value(_OUTPUT_PATH),
      "Optional: Path for the output bag file.")(
      "include-depth-map,m",
      po::value<bool>()->default_value(_INCLUDE_DEPTH_MAP),
      "Optional: Publish a depth map version of the lidar data.");

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

  const auto data_path = *data_path_opt;
  const auto output_path = vm["output-path"].as<std::string>();
  const auto include_depth_map = vm["include-depth-map"].as<bool>();

  boost::filesystem::path d(data_path);
  const auto timestamp = d.parent_path().parent_path().filename().string();

  const auto file_basename =
      (timestamp + "_" + boost::filesystem::basename(data_path));
  const auto topic_prefix =
      (std::string(_DATASET_NAMESPACE) + "/" + file_basename);

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
  /// Load each npz file, convert to PointCloud2 message, write to bag
  ///

  const auto fields = a2d2_to_ros::get_npz_fields();

  rosbag::Bag bag;
  const auto bag_name = output_path + "/" + file_basename + ".bag";
  bag.open(bag_name, rosbag::bagmode::Write);
  for (const auto& f : files) {
    ///
    /// Load and verify the data
    ///

    cnpy::npz_t npz;
    try {
      npz = cnpy::npz_load(f);
    } catch (const std::exception& e) {
      ROS_FATAL_STREAM(e.what());
      bag.close();
      return EXIT_FAILURE;
    }

    const auto npz_structure_valid = a2d2_to_ros::verify_npz_structure(npz);
    if (!npz_structure_valid) {
      ROS_FATAL_STREAM(
          "Encountered unexpected structure in the data. Cannot continue.");
      bag.close();
      return EXIT_FAILURE;
    } else {
      ROS_INFO_STREAM("Successfully loaded npz data from:\n" << f);
    }

    // for convenience
    const auto& points = npz[fields[a2d2_to_ros::lidar::POINTS_IDX]];
    const auto& azimuth = npz[fields[a2d2_to_ros::lidar::AZIMUTH_IDX]];
    const auto& boundary = npz[fields[a2d2_to_ros::lidar::BOUNDARY_IDX]];
    const auto& col = npz[fields[a2d2_to_ros::lidar::COL_IDX]];
    const auto& depth = npz[fields[a2d2_to_ros::lidar::DEPTH_IDX]];
    const auto& distance = npz[fields[a2d2_to_ros::lidar::DISTANCE_IDX]];
    const auto& lidar_id = npz[fields[a2d2_to_ros::lidar::ID_IDX]];
    const auto& rectime = npz[fields[a2d2_to_ros::lidar::RECTIME_IDX]];
    const auto& reflectance = npz[fields[a2d2_to_ros::lidar::REFLECTANCE_IDX]];
    const auto& row = npz[fields[a2d2_to_ros::lidar::ROW_IDX]];
    const auto& timestamp = npz[fields[a2d2_to_ros::lidar::TIMESTAMP_IDX]];
    const auto& valid = npz[fields[a2d2_to_ros::lidar::VALID_DIX]];

    ///
    /// Build pointcloud message
    ///

    for (auto r = 0; r < points.shape[0]; ++r) {
      const auto data = points.data<double>();
      for (auto c = 0; c < points.shape[1]; ++c) {
        const auto idx = a2d2_to_ros::flatten_2d_index(points.shape[1], r, c);
        //        std::cout << data[idx] << ", ";
      }
      //      std::cout << std::endl;
    }

    break;
  }

  bag.close();
  return EXIT_SUCCESS;
}
