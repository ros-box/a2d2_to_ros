# Walkthrough

This doc gives an example walkthrough of using the data set converter. See [README.md](README.md) for more details.

> Note that the provided RViz config visualizes only camera and lidar data, no bus signal data.

## Prerequisites

This package has the following dependencies in addition to standard ROS dependencies:

* [RapidJSON](https://rapidjson.org/): used to load, parse, and validate the JSON data files
* [ROS CNPY](https://gitlab.com/MaplessAI/external/ros_cnpy): used to load `.npz` files for lidar data

The ROS CNPY package can be downloaded at the above link, and RapidJSON can be installed with:

```console
$ rosdep install a2d2_to_ros --ignore-src -r -y
```

Ensure all packages are built.

## PLEASE NOTE

When specifying a location (i.e., directory) as an argument, you probably do not want to use a trailing slash, e.g.:

```
--camera-data-path /foo/bar    # do this
--camera-data-path /foo/bar/   # not this
```

The converters traverse the directory hierarchies to extract information from the directory names, and including a trailing slash changes the behavior of that traversal in (likely) unintended ways. See <https://www.boost.org/doc/libs/1_60_0/libs/filesystem/doc/reference.html#path-parent_path>.

## Convert and visualize Front Center sensor data

For the example below, assume the following locations:

* Package: `~/catkin_ws/src/a2d2_to_ros`
* Data set: `~/data/a2d2/Ingolstadt`

To convert and visualize data from the front center sensors:

* Download "Bus Signals", "Camera – Front Center", and "Lidar – Front Center" from one of the cities from <https://www.a2d2.audi/a2d2/en/download.html>
* Convert the camera data:

```console
$ rosrun a2d2_to_ros sensor_fusion_camera --camera-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/camera/cam_front_center --frame-info-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_camera_frame.schema --sensor-config-path ~/data/a2d2/cams_lidars.json --sensor-config-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_config.schema
```

This outputs the bag file: `20190401_145936_cam_front_center_camera.bag`

* Convert the lidar data:

```console
$ rosrun a2d2_to_ros sensor_fusion_lidar --lidar-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/lidar/cam_front_center --camera-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/camera/cam_front_center --frame-info-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_camera_frame.schema
```

This outputs the bag file: `20190401_145936_cam_front_center_lidar.bag`

* Convert bus signal data:

```console
$ rosrun a2d2_to_ros sensor_fusion_bus_signals --sensor-config-json-path ~/data/a2d2/cams_lidars.json --sensor-config-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_config.schema --bus-signal-json-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/bus/20190401145936_bus_signals.json --bus-signal-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_bus_signal.schema
```

This outputs the bag files:

```
20190401145936_bus_signals.bag
20190401145936_bus_signals_tf.bag
```

* Launch RViz to visualize (ensure a roscore is running)

```console
$ roslaunch a2d2_to_ros visualize.launch
```

* Play back files to visualize in RViz

```console
$ rosbag play 20190401145936_bus_signals_tf.bag 20190401_145936_cam_front_center_camera.bag 20190401_145936_cam_front_center_lidar.bag
```
