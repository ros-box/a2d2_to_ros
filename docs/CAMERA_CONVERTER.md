# Converter: Sensor Fusion > Camera

This converter parses camera frame data converts each to a `sensor_msgs::Image` message.

## Usage

An example invocation is given below. For the example, assume the following locations:

* Package: `~/catkin_ws/src/a2d2_to_ros`
* Data set: `~/data/a2d2/Ingolstadt`

```console
$ rosrun a2d2_to_ros sensor_fusion_camera --camera-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/camera/cam_front_left --frame-info-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_camera_frame.schema --verbose true
```

This command will create the following bag file:

```console
./20190401_145936_cam_front_left_camera.bag
```

To get a full list of usage options, run with the `--help` switch:

```console
$ rosrun a2d2_to_ros sensor_fusion_camera --help
---Built with stream logging enabled.
---Built to use single precision for float values.
Convert sequential camera data to rosbag for the A2D2 Sensor Fusion data set. See README.md for details.
Available options are listed below. Arguments without default values are required:
  -h [ --help ]                                    Print help and exit.
  -c [ --camera-data-path ] arg                    Path to the camera data files.
  -s [ --frame-info-schema-path ] arg              Path to the JSON schema for camera frame info files.
  -t [ --include-clock-topic ] arg (=0)            Optional: Use timestamps from the data to write a /clock topic.
  -m [ --min-time-offset ] arg (=0)                Optional: Seconds to skip ahead in the data before starting the bag.
  -d [ --duration ] arg (=1.7976931348623157e+308) Optional: Seconds after min-time-offset to include in bag file.
  -o [ --output-path ] arg (=.)                    Optional: Path for the output bag file.
  -v [ --verbose ] arg (=0)                        Optional: Show name of each file after it is processed.
```
