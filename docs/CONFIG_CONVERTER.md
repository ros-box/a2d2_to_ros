# Converter: Sensor Fusion > Sensor Configuration

This converter parses the `cams_lidars.json` sensor config file and publishes the corresponding [TF2](http://wiki.ros.org/tf2) tree.

## PLEASE NOTE

When specifying a location (i.e., directory) as an argument, you probably do not want to use a trailing slash, e.g.:

```
--sensor-config-path /foo/bar    # do this
--sensor-condig-path /foo/bar/   # not this
```

The converters traverse the directory hierarchies to extract information from the directory names, and including a trailing slash changes the behavior of that traversal in (likely) unintended ways. See <https://www.boost.org/doc/libs/1_60_0/libs/filesystem/doc/reference.html#path-parent_path>.

### Usage

An example invocation is given below. For the example, assume the following locations:

* Package: `~/catkin_ws/src/a2d2_to_ros`
* Data set: `~/data/a2d2/Ingolstadt`

```console
$ rosrun a2d2_to_ros sensor_fusion_config --sensor-config-path ~/data/a2d2/cams_lidars.json --sensor-config-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_config.schema --reference-bag-path ~/catkin_ws/20190401_145936_cam_front_center_lidar.bag
```

This command will create the following bag file, whose name is the same as the reference bag with "\_tf" appended to the basename:

```console
./20190401_145936_cam_front_center_lidar_tf.bag
```

The time span that TF messages are published for is taken from the bag file given by `--reference-bag-path` argument. So, for instance, if the lidar bags are generated, this utility can then be used to generate a corresponding TF bag file.

To get a full list of usage options, run with the `--help` switch:

```console
$ rosrun a2d2_to_ros sensor_fusion_config --help
---Built with stream logging enabled.
---Built to use single precision for float values.
Write a transform bag file containing the vehicle box model and tf tree for the vehicle sensor configuration. The bag is written with respect to the begin and end times of a reference bag file. This means lidar and camera bag files can be generated first, then this utility can be used to generate a tf bag file for each of them.:
  -h [ --help ]                          Print help and exit.
  -c [ --sensor-config-path ] arg        Path to the JSON for vehicle/sensor config.
  -s [ --sensor-config-schema-path ] arg Path to the JSON schema for the vehicle/sensor config.
  -r [ --reference-bag-path ] arg        Path to the reference bag file containing the desired time span.
  -f [ --tf-frequency ] arg (=10)        Optional: Publish frequency for tf transforms and ego shape message.
  -o [ --output-path ] arg (=.)          Optional: Path for the output bag file.
```
