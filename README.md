# A2D2 to ROS

![Front center lidar with TF tree](media/screenshot.png "Front center lidar with TF tree")

Utilities for converting [A2D2 data sets](https://www.a2d2.audi/) to ROS bags.

The idea is that there is an executuable for each sensor modality: camera, lidar, and bus. Bag files are generated for these modalities independently.

For a step-by-step example of how to use the converters, see [WALKTHROUGH.md](WALKTHROUGH.md).

## Requirements

This package has the following dependencies in addition to standard ROS dependencies:

* [RapidJSON](https://rapidjson.org/): used to load, parse, and validate the JSON data files
* [ROS CNPY](https://gitlab.com/MaplessAI/external/ros_cnpy): used to load `.npz` files for lidar data

The ROS CNPY package can be downloaded at the above link, and RapidJSON can be installed with:

```console
$ rosdep install a2d2_to_ros --ignore-src -r -y
```

## FAQ

[FAQ.md](FAQ.md) contains common questions about the A2D2 data set.

## Converters

* [Sensor Fusion > Camera](docs/CAMERA_CONVERTER.md)
* [Sensor Fusion > Sensor Configuration](docs/CONFIG_CONVERTER.md)
* [Sensor Fusion > Lidar](docs/LIDAR_CONVERTER.md)
* [Sensor Fusion > Bus Signal](docs/BUS_SIGNAL_CONVERTER.md)

## Visualization

An example RViz config is included along a convenience launch file:

```console
$ roslaunch a2d2_to_ros visualize.launch
```

This launches RViz pre-configured to visualize the TF tree, and the front-facing camera and lidar data.

## Compatibility

This code is built and tested under:

* [ROS Melodic](https://wiki.ros.org/melodic) with [Ubuntu 18.04.4](http://releases.ubuntu.com/18.04/)
* [Clang 6.0.0](https://releases.llvm.org/6.0.0/tools/clang/docs/ReleaseNotes.html) with `-std=c++14`

There is nothing very platform specific, so other reasonably similar system configurations should work.

## TODO

A list of open issues for the converter can be found [here](https://gitlab.com/MaplessAI/external/a2d2_to_ros/-/issues).
