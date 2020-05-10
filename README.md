# A2D2 to ROS

Utilities for converting [A2D2 data sets](https://www.a2d2.audi/) to ROS bags.

The idea is that there is an executuable for each sensor modality: camera, lidar, and bus. Bag files are generated for these modalities independently.

NOTE: Currently, there is only a converter for the [Sensor Fusion > Bus Signal](https://www.a2d2.audi/a2d2/en/download.html) data sets.

## Requirements

This package has the following dependencies in addition to standard ROS dependencies:

* [RapidJSON](https://rapidjson.org/): used to load, parse, and validate the JSON data files
* [ROS CNPY](https://gitlab.com/MaplessAI/external/ros_cnpy): used to load `.npz` files for lidar data

The ROS CNPY package can be downloaded at the above link, and RapidJSON can be installed with:

```
$ rosdep install a2d2_to_ros --ignore-src -r -y
```

## FAQ

[FAQ.md](FAQ.md) contains common questions about the A2D2 data set.

## Converter: Sensor Fusion > Lidar

This converter parses lidar frame data saved in numpy format and converts each frame to a `sensor_msgs::PointCloud2` message.

```
$ rosrun a2d2_to_ros sensor_fusion_lidar --lidar-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/lidar/cam_front_center --camera-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/camera/cam_front_center --frame-info-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_camera_frame.schema --verbose true
```

### Type conversions

In the A2D2 data set, floating point types are stored with 64-bit width. However, this much precision is typically not needed (and possibly not even necessary, see [FAQ.md](FAQ.md)).For that reason, this package converts 64-bit floating point information to 32-bit. If 64-bit width is desired, edit the [CMakelists.txt](CMakelists.txt) file to uncomment the `-DUSE_FLOAT64` compile definition:

```
# uncomment to use 64-bit instead of 32-bit width for floats in point cloud;
# really need full precision, best to leave it 32-bit
#add_definitions(-DUSE_FLOAT64)
```

As noted in the cmake file, this can cause ROS tools, such as RViz, to be unable to interpret the messages.

Additionally, in the interest of saving space where possible, integer and bool fields use smaller width data types. Unlike the float values, however, this does not result in potential loss of information because the full integer widths are never used.

The full mapping of types is provided in the table below. "A2D2 type" is the type used by numpy to store the data, "PointCloud2 type" is the type used to write data to the point cloud message (for this the type itself is not so important as the width), and "Interpretation type" is the type the value should be case to when retrieving the value from the message. Programmatically the type conversion information is available in the `ReadTypes` and `WriteTypes` structs in [includes/lib\_a2d2\_to\_ros.hpp](includes/a2d2_to_ros/lib_a2d2_to_ros.hpp):

| A2D2 field               | A2D2 type | PointCloud2 type                        | Interpretation type |
|--------------------------|:---------:|:---------------------------------------:|--------------------:|
| pcloud\_points           | `float64` | `sensor_msgs::PointField::FLOAT(32|64)` | `float` or `double` |
| pcloud\_attr.col         | `float64` | `sensor_msgs::PointField::FLOAT(32|64)` | `float` or `double` |
| pcloud\_attr.depth       | `float64` | `sensor_msgs::PointField::FLOAT(32|64)` | `float` or `double` |
| pcloud\_attr.distance    | `float64` | `sensor_msgs::PointField::FLOAT(32|64)` | `float` or `double` |
| pcloud\_attr.row         | `float64` | `sensor_msgs::PointField::FLOAT(32|64)` | `float` or `double` |
| pcloud\_attr.rectime     | `int64`   | `sensor_msgs::PointField::FLOAT64`      | `uint64_t`          |
| pcloud\_attr.timestamp   | `int64`   | `sensor_msgs::PointField::FLOAT64`      | `uint64_t`          |
| pcloud\_attr.lidar\_id   | `int64`   | `sensor_msgs::PointField::UINT8`        | `uint8_t`           |
| pcloud\_attr.reflectance | `int64`   | `sensor_msgs::PointField::UINT8`        | `uint8_t`           |
| pcloud\_attr.boundary    | `int64`   | `sensor_msgs::PointField::UINT8`        | `bool`              |
| pcloud\_attr.valid       | `int64`   | `sensor_msgs::PointField::UINT8`        | `bool`              |



```
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"
namespace a2d2 = a2d2_to_ros;

const auto fields = a2d2::get_npz_fields();
auto iters = a2d2::A2D2_PointCloudIterators(msg, fields);
for (auto row = 0; row < n_points; ++row, ++iters) {
  // the below is equivalent to: std::cout << iters << "\n";
  std::cout << "{"
     << "x: " << *(iters.x)
     << ", y: " << *(iters.y)
     << ", z: " << *(iters.z)
     << ", azimuth: " << *(iters.azimuth)
     << ", boundary: " << static_cast<bool>(*(iters.boundary))
     << ", col: " << *(iters.col)
     << ", depth: " << *(iters.depth)
     << ", distance: " << *(iters.distance)
     << ", lidar_id: " << static_cast<int>(*(iters.lidar_id))
     << ", rectime: " << *(iters.rectime)
     << ", reflectance: " << static_cast<int>(*(iters.reflectance))
     << ", row: " << *(iters.row)
     << ", timestamp: " << *(iters.timestamp)
     << ", valid: " << static_cast<bool>(*(iters.valid))
     << "}\n";
}
```

### RViz config

An example RViz config is included along a convenience launch file:

```
roslaunch a2d2_to_ros viz_lidar_frontcenter.launch
```

The config only includes the `front_center` lidar, but the settings should work for any of them.

## Converter: Sensor Fusion > Bus Signal

This converter parses a bus signal data JSON file and outputs the data into a bag file.

### JSON validation

A [JSON Schema](http://json-schema.org/) file is provided in [schemas/sensor\_fusion\_bus\_signal.schema](schemas/sensor_fusion_bus_signal.schema) to perform validation.

### Usage

An example invocation is given below. For the example, assume the following locations:

* Package: `~/catkin_ws/src/a2d2_to_ros`
* Data set: `~/data/a2d2/Munich`

```
$ rosrun a2d2_to_ros sensor_fusion_bus_signals --schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_bus_signal.schema --json-path ~/data/a2d2/Munich/camera_lidar/20190401_121727/bus/20190401121727_bus_signals.json
```

This command will create the following bag file:

```
~/catkin_ws/src/a2d2_to_ros/20190401121727_bus_signals.bag
```

To get a full list of usage options, run with the `--help` switch:

```
$ rosrun a2d2_to_ros sensor_fusion_bus_signals --help
Convert sequential bus signal data to rosbag for the A2D2 Sensor Fusion data set. See README.md for details.
Available options are listed below. Arguments without default values are required:
  -h [ --help ]                              Print help and exit.
  -s [ --schema-path ] arg                   Path to the JSON schema.
  -j [ --json-path ] arg                     Path to the JSON data set file.
  -o [ --output-path ] arg (=.)              Optional: Path for the output bag file.
  -b [ --bus-frame-name ] arg (=bus)         Optional: Frame name to use for bus signals.
  -v [ --include-original-values ] arg (=1)  Optional: Include the original data set values in their original units.
  -r [ --include-converted-values ] arg (=1) Optional: Include data set values converted to ROS standard units.
```

### Bag file conventions

* Each field in the JSON file corresponds to up to four topics in the generated bag:
    * `original_value`: The original value recorded in the JSON file as a [std\_msgs::Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html) message
    * `original_units`: The units of the original value recorded in the JSON file as a [std\_msgs::String](http://docs.ros.org/api/std_msgs/html/msg/String.html) message (this is published only once per bag file).
    * `value`: The value in the JSON file converted to ROS standard units as a [std\_msgs::Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html) message
    * `header`: A [std\_msgs::Header](https://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html) message that contains the timestamp of the recorded data
* The topics under each field name are guaranteed to all have the same header time stamp and message time
* The `original_value` and `original_units` topics are not included if the converter is run with `--include-original-values false`
* The `value` topic is not included if the converter is run with `--include-converted-values false`
* The message time in the bag file is the same as the timestamp in the header message.
* Each bag file contains a `/clock` topic that has a [rosgraph\_msgs::Clock](http://docs.ros.org/api/rosgraph_msgs/html/msg/Clock.html) message for each unique timestamp in the data set.
* The output bag file is given the same basename as the input JSON file.
* Each of the topics in the bag file (except for `/clock`) is prefixed with `/a2d2/[JSON_FILE_BASENAME]`

## Compatibility

This code is built and tested under:

* [ROS Melodic](https://wiki.ros.org/melodic) with [Ubuntu 18.04.4](http://releases.ubuntu.com/18.04/)
* [Clang 6.0.0](https://releases.llvm.org/6.0.0/tools/clang/docs/ReleaseNotes.html) with `-std=c++14`

There is nothing very platform specific, so other reasonably similar system configurations should work.
