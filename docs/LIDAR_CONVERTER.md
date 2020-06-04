# Converter: Sensor Fusion > Lidar

This converter parses lidar frame data saved in numpy format and converts each frame to a `sensor_msgs::PointCloud2` message.

## PLEASE NOTE

When specifying a location (i.e., directory) as an argument, you probably do not want to use a trailing slash, e.g.:

```
--camera-data-path /foo/bar    # do this
--camera-data-path /foo/bar/   # not this
```

The converters traverse the directory hierarchies to extract information from the directory names, and including a trailing slash changes the behavior of that traversal in (likely) unintended ways. See <https://www.boost.org/doc/libs/1_60_0/libs/filesystem/doc/reference.html#path-parent_path>.

## Usage

An example invocation is given below. For the example, assume the following locations:

* Package: `~/catkin_ws/src/a2d2_to_ros`
* Data set: `~/data/a2d2/Ingolstadt`

```console
$ rosrun a2d2_to_ros sensor_fusion_lidar --lidar-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/lidar/cam_front_center --camera-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/camera/cam_front_center --frame-info-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_camera_frame.schema --verbose true
```

This command will create the following bag file:

```console
./20190401_145936_cam_front_center_lidar.bag
```

To get a full list of usage options, run with the `--help` switch:

```console
$ rosrun a2d2_to_ros sensor_fusion_lidar --help
<Lidar Converter>
---Built with stream logging enabled.
---Built to use single precision for float values.
Convert sequential lidar data to rosbag for the A2D2 Sensor Fusion data set. See README.md for details.
Available options are listed below. Arguments without default values are required:
  -h [ --help ]                                    Print help and exit.
  -d [ --lidar-data-path ] arg                     Path to the lidar data files.
  -c [ --camera-data-path ] arg                    Path to the camera data files (for timestamp information).
  -s [ --frame-info-schema-path ] arg              Path to the JSON schema for camera frame info files.
  -t [ --include-clock-topic ] arg (=0)            Optional: Use timestamps from the data to write a /clock topic.
  -a [ --start-time ] arg (=0)                     Optional: Start on or after this time (TAI microseconds).
  -m [ --min-time-offset ] arg (=0)                Optional: Seconds to skip ahead in the data before starting the bag.
  -d [ --duration ] arg (=1.7976931348623157e+308) Optional: Seconds after min-time-offset to include in bag file.
  -o [ --output-path ] arg (=.)                    Optional: Path for the output bag file.
  -i [ --include-depth-map ] arg (=0)              Optional: Publish a depth map version of the lidar data.
  -v [ --verbose ] arg (=0)                        Optional: Show name of each file after it is processed.
```

## Type conversions

In the A2D2 data set, floating point types are stored with double precision. However, this much precision is not necessary (see [docs/FAQ.md](docs/FAQ.md)). For that reason, this package converts double precision floating point information to single. Additionally, in the interest of saving space where possible, integer and bool fields use smaller width data types.

The full mapping of types is provided in the table below. **A2D2 type** is the type used by numpy to store the data, **PointCloud2 type** is the type used to write data to the point cloud message (for this the type itself is not so important as the width), and **Use type** is the type the value should be interpreted as when retrieving it from the message. Programmatically the type conversion information is available in the `ReadTypes` and `WriteTypes` structs in [include/a2d2\_to\_ros/npz.hpp](include/a2d2_to_ros/npz.hpp):

| A2D2 field                 | A2D2 type | PointCloud2 type                   | Use type   |
|----------------------------|:---------:|:----------------------------------:|-----------:|
| *pcloud\_points*           | `float64` | `sensor_msgs::PointField::FLOAT32` | `float`    |
| *pcloud\_attr.col*         | `float64` | `sensor_msgs::PointField::FLOAT32` | `float`    |
| *pcloud\_attr.depth*       | `float64` | `sensor_msgs::PointField::FLOAT32` | `float`    |
| *pcloud\_attr.distance*    | `float64` | `sensor_msgs::PointField::FLOAT32` | `float`    |
| *pcloud\_attr.row*         | `float64` | `sensor_msgs::PointField::FLOAT32` | `float`    |
| *pcloud\_attr.rectime*     | `int64`   | `sensor_msgs::PointField::FLOAT64` | `uint64_t` |
| *pcloud\_attr.timestamp*   | `int64`   | `sensor_msgs::PointField::FLOAT64` | `uint64_t` |
| *pcloud\_attr.lidar\_id*   | `int64`   | `sensor_msgs::PointField::UINT8`   | `uint8_t`  |
| *pcloud\_attr.reflectance* | `int64`   | `sensor_msgs::PointField::UINT8`   | `uint8_t`  |
| *pcloud\_attr.boundary*    | `int64`   | `sensor_msgs::PointField::UINT8`   | `bool`     |
| *pcloud\_attr.valid*       | `int64`   | `sensor_msgs::PointField::UINT8`   | `bool`     |

An iterator struct is provided for convenience to programmatically store and retrieve the data in the point clouds. An example of using the iterator struct is given below:

```cpp
#include "a2d2_to_ros/lib_a2d2_to_ros.hpp"
namespace a2d2 = a2d2_to_ros;

// Assume 'msg' is the PointCloud2 message

const auto fields = a2d2::npz::Fields::get_fields();
auto iters = a2d2::A2D2_PointCloudIterators(msg, fields);
for (auto row = 0; row < msg.width; ++row, ++iters) {
  // the below is equivalent to: std::cout << iters << "\n";
  std::cout << "{"
     << "x: " << *(iters.x)
     << ", y: " << *(iters.y)
     << ", z: " << *(iters.z)
     << ", azimuth: " << *(iters.azimuth)
     << ", boundary: " << *(iters.boundary)
     << ", col: " << *(iters.col)
     << ", depth: " << *(iters.depth)
     << ", distance: " << *(iters.distance)
     << ", lidar_id: " << static_cast<int>(*(iters.lidar_id))
     << ", rectime: " << *(iters.rectime)
     << ", reflectance: " << static_cast<int>(*(iters.reflectance))
     << ", row: " << *(iters.row)
     << ", timestamp: " << *(iters.timestamp)
     << ", valid: " << *(iters.valid)
     << "}\n";
}
```
## Bag file conventions

* The message time in the bag file is the same as the timestamp in the header message.
* The output bag file is given the same basename as the directory containing the `.npz` files.
* Each of the topics in the bag file (except for `/clock`) is prefixed with `/a2d2/[RECORD_TIME]`
