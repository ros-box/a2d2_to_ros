# Converter: Sensor Fusion > Bus Signal

This converter parses a bus signal data JSON file and outputs the data into a bag file. In addition, this converter creates a bag file that publishes TF data for the vehicle along with the vehicle box model.

## JSON validation

A [JSON Schema](http://json-schema.org/) file is provided to perform validation:

* [schemas/sensor\_fusion\_bus\_signal.schema](schemas/sensor_fusion_bus_signal.schema): validates the bus signal JSON file

As of this writing, RapidJSON validates against [JSON Schema draft 04](https://rapidjson.org/md_doc_schema.html#Conformance).

## PLEASE NOTE

When specifying a location (i.e., directory) as an argument, you probably do not want to use a trailing slash, e.g.:

```
--schema-path /foo/bar    # do this
--schema-path /foo/bar/   # not this
```

The converters traverse the directory hierarchies to extract information from the directory names, and including a trailing slash changes the behavior of that traversal in (likely) unintended ways. See <https://www.boost.org/doc/libs/1_60_0/libs/filesystem/doc/reference.html#path-parent_path>.

## Usage

An example invocation is given below. For the example, assume the following locations:

* Package: `~/catkin_ws/src/a2d2_to_ros`
* Data set: `~/data/a2d2/Munich`

```console
$ rosrun a2d2_to_ros sensor_fusion_bus_signals --sensor-config-json-path ~/data/a2d2 --sensor-config-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_config.schema --bus-signal-json-path ~/data/a2d2/Ingolstadt/camera_lidar --bus-signal-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_bus_signal.schema
```

This command will create the following bag files:

```console
./20190401121727_bus_signals.bag
./20190401121727_bus_signals_tf.bag
```

To get a full list of usage options, run with the `--help` switch:

```console
$ rosrun a2d2_to_ros sensor_fusion_bus_signals --help
<Bus Signal Converter>
---Built with stream logging enabled.
---Built to use single precision for float values.
Convert sequential bus signal data to rosbag for the A2D2 Sensor Fusion data set. In addition, write a transform bag file containing the vehicle box model and tf tree for the vehicle sensor configuration. See README.md for details.
Available options are listed  below. Arguments without default values are required:
  -h [ --help ]                                    Print help and exit.
  -c [ --sensor-config-json-path ] arg             Path to the directory containing the JSON for vehicle/sensor config.
  -s [ --sensor-config-schema-path ] arg           Path to the JSON schema for the vehicle/sensor config.
  -j [ --bus-signal-json-path ] arg                Path to the directory containing the JSON bus signal file.
  -b [ --bus-signal-schema-path ] arg              Path to the JSON schema for bus signal data.
  -a [ --start-time ] arg (=0)                     Optional: Start on or after this time (TAI microseconds).
  -m [ --min-time-offset ] arg (=0)                Optional: Seconds to skip ahead in the data before starting the bag.
  -d [ --duration ] arg (=1.7976931348623157e+308) Optional: Seconds after min-time-offset to include in bag file.
  -o [ --output-path ] arg (=.)                    Optional: Path for the output bag file.
  -t [ --include-clock-topic ] arg (=0)            Optional: Write bus signal times to a /clock topic in the TF bag.
  -i [ --include-original-values ] arg (=0)        Optional: Include data set values in their original units.
  -r [ --include-converted-values ] arg (=1)       Optional: Include data set values converted to ROS standard units.
  -v [ --verbose ] arg (=0)                        Optional: Show name of each file after it is processed.
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
* The optional `/clock` topic in the TF bag has a [rosgraph\_msgs::Clock](http://docs.ros.org/api/rosgraph_msgs/html/msg/Clock.html) message for each unique timestamp in the data set.
* The output bag file is given the same basename as the input JSON file.
* Each of the topics in the bag file (except for `/clock`) is prefixed with `/a2d2/[JSON_FILE_BASENAME]`
