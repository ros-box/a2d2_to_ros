# Converter: Sensor Fusion > Bus Signal

This converter parses a bus signal data JSON file and outputs the data into a bag file.

## JSON validation

A [JSON Schema](http://json-schema.org/) file is provided to perform validation:

* [schemas/sensor\_fusion\_bus\_signal.schema](schemas/sensor_fusion_bus_signal.schema): validates the bus signal JSON file

As of this writing, RapidJSON validates against [JSON Schema draft 04](https://rapidjson.org/md_doc_schema.html#Conformance).

## Usage

An example invocation is given below. For the example, assume the following locations:

* Package: `~/catkin_ws/src/a2d2_to_ros`
* Data set: `~/data/a2d2/Munich`

```console
$ rosrun a2d2_to_ros sensor_fusion_bus_signals --schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_bus_signal.schema --json-path ~/data/a2d2/Munich/camera_lidar/20190401_121727/bus/20190401121727_bus_signals.json
```

This command will create the following bag file:

```console
./20190401121727_bus_signals.bag
```

To get a full list of usage options, run with the `--help` switch:

```console
$ rosrun a2d2_to_ros sensor_fusion_bus_signals --help
---Built with stream logging enabled.
---Built to use single precision for float values.
Convert sequential bus signal data to rosbag for the A2D2 Sensor Fusion data set. See README.md for details.
Available options are listed below. Arguments without default values are required:
  -h [ --help ]                                    Print help and exit.
  -s [ --schema-path ] arg                         Path to the JSON schema.
  -j [ --json-path ] arg                           Path to the JSON data set file.
  -m [ --min-time-offset ] arg (=0)                Optional: Seconds to skip ahead in the data before starting the bag.
  -d [ --duration ] arg (=1.7976931348623157e+308) Optional: Seconds after min-time-offset to include in bag file.
  -o [ --output-path ] arg (=.)                    Optional: Path for the output bag file.
  -c [ --include-clock-topic ] arg (=0)            Optional: Use timestamps from the data to write a /clock topic.
  -v [ --include-original-values ] arg (=0)        Optional: Include data set values in their original units.
  -r [ --include-converted-values ] arg (=1)       Optional: Include data set values converted to ROS standard units.
```

## Bag file conventions

* Each field in the JSON file corresponds to up to four topics in the generated bag:
    * `original_value`: The original value recorded in the JSON file as a [std\_msgs::Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html) message
    * `original_units`: The units of the original value recorded in the JSON file as a [std\_msgs::String](http://docs.ros.org/api/std_msgs/html/msg/String.html) message (this is published only once per bag file).
    * `value`: The value in the JSON file converted to ROS standard units as a [std\_msgs::Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html) message
    * `header`: A [std\_msgs::Header](https://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html) message that contains the timestamp of the recorded data
* The topics under each field name are guaranteed to all have the same header time stamp and message time
* The `original_value` and `original_units` topics are not included if the converter is run with `--include-original-values false`
* The `value` topic is not included if the converter is run with `--include-converted-values false`
* The message time in the bag file is the same as the timestamp in the header message.
* Each bag file contains a `/clock` topic that has a [rosgraph\_msgs::Clock](http://docs.ros.org/api/rosgraph_msgs/html/msg/Clock.html) message for each unique timestamp in the data set.
* The output bag file is given the same basename as the input JSON file.
* Each of the topics in the bag file (except for `/clock`) is prefixed with `/a2d2/[JSON_FILE_BASENAME]`
