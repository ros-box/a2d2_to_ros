# Walkthrough

This doc gives an example walkthrough of using the data set converter.

## Prerequisites

This package has the following dependencies in addition to standard ROS dependencies:

* [RapidJSON](https://rapidjson.org/): used to load, parse, and validate the JSON data files
* [ROS CNPY](https://gitlab.com/MaplessAI/external/ros_cnpy): used to load `.npz` files for lidar data

The ROS CNPY package can be downloaded at the above link, and RapidJSON can be installed with:

```console
$ rosdep install a2d2_to_ros --ignore-src -r -y
```

Ensure all packages are built.

## Convert and visualize Front Center sensor data

For the example below, assume the following locations:

* Package: `~/catkin_ws/src/a2d2_to_ros`
* Data set: `~/data/a2d2/Ingolstadt`

To convert and visualize data from the front center sensors:

* Download "Camera – Front Center" and "Lidar – Front Center" from one of the cities from [https://www.a2d2.audi/a2d2/en/download.html](https://www.a2d2.audi/a2d2/en/download.html)
* Convert the camera data:

```console
$ rosrun a2d2_to_ros sensor_fusion_camera --camera-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/camera/cam_front_center --frame-info-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_camera_frame.schema
```

    * This outputs the bag file: `20190401_145936_cam_front_center_camera.bag`

* Convert the lidar data:

```console
$ rosrun a2d2_to_ros sensor_fusion_lidar --lidar-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/lidar/cam_front_center --camera-data-path ~/data/a2d2/Ingolstadt/camera_lidar/20190401_145936/camera/cam_front_center --frame-info-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_fusion_camera_frame.schema
```
This outputs the bag file: `20190401_145936_cam_front_center_lidar.bag`

* Convert TF data (using the camera bag as reference):

```console
$ rosrun a2d2_to_ros sensor_fusion_config --sensor-config-path ~/data/a2d2/cams_lidars.json --sensor-config-schema-path ~/catkin_ws/src/a2d2_to_ros/schemas/sensor_config.schema --reference-bag-path 20190401_145936_cam_front_center.bag
```
This outputs the bag file: `20190401_145936_cam_front_center_camera_tf.bag`

* Launch RViz to visualize (ensure a roscore is running)

```console
$ roslaunch a2d2_to_ros visualize.launch
```

* Play back files to visualize in RViz

```console
$ rosbag play 20190401_145936_cam_front_center_camera.bag 20190401_145936_cam_front_center_camera_tf.bag 20190401_145936_cam_front_center_lidar.bag
```
Note that the provided RViz config visualizes all sensors but this walkthrough only converts and visualizes the front center sensors.
