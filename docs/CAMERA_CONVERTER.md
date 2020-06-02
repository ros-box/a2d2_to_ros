# Converter: Sensor Fusion > Camera

This converter parses camera frame data converts each to a [sensor\_msgs::Image](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) message. In addition, the camera calibration data is published in a [sensor\_msgs::CameraInfo](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html) message.

## JSON validation

Two [JSON Schema](http://json-schema.org/) files are provided to perform validation of the data set file:

* [schemas/sensor\_config.schema](schemas/sensor_config.schema): validates configuration file `cams_lidars.json`, which contains camera calibration information
* [schemas/sensor\_fusion\_camera\_frame.schema](schemas/sensor_fusion_camera_frame.schema): validates the frame info JSON files associated with each camera frame, which contains timestamp information

As of this writing, RapidJSON validates against [JSON Schema draft 04](https://rapidjson.org/md_doc_schema.html#Conformance).

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
$ rosrun a2d2_to_ros sensor_fusion_camera --camera-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/camera/cam_front_center --frame-info-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_camera_frame.schema --sensor-config-path ~/data/a2d2/cams_lidars.json --sensor-config-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_config.schema
```

This command will create the following bag file:

```console
./20190401_145936_cam_front_center_camera.bag
```

To get a full list of usage options, run with the `--help` switch:

```console
$ rosrun a2d2_to_ros sensor_fusion_camera --help
<Camera Converter>
---Built with stream logging enabled.
---Built to use single precision for float values.
Convert sequential camera data to rosbag for the A2D2 Sensor Fusion data set. See README.md for details.
Available options are listed below. Arguments without default values are required:
  -h [ --help ]                                    Print help and exit.
  -c [ --camera-data-path ] arg                    Path to the camera data files.
  -f [ --frame-info-schema-path ] arg              Path to the JSON schema for camera frame info files.
  -p [ --sensor-config-path ] arg                  Path to the JSON for vehicle/sensor config.
  -s [ --sensor-config-schema-path ] arg           Path to the JSON schema for the vehicle/sensor config.
  -t [ --include-clock-topic ] arg (=0)            Optional: Use timestamps from the data to write a /clock topic.
  -a [ --start-time ] arg (=0)                     Optional: Start on or after this time (TAI microseconds).
  -m [ --min-time-offset ] arg (=0)                Optional: Seconds to skip ahead in the data before starting the bag.
  -d [ --duration ] arg (=1.7976931348623157e+308) Optional: Seconds after min-time-offset to include in bag file.
  -o [ --output-path ] arg (=.)                    Optional: Path for the output bag file.
  -v [ --verbose ] arg (=0)                        Optional: Show name of each file after it is processed.
```

## Bag file conventions

* The message time in the bag file is the same as the timestamp in the header message.
* The output bag file is given the same basename as the input JSON file.
* Each of the topics in the bag file (except for `/clock`) is prefixed with `/a2d2/[RECORD_TIME]`
