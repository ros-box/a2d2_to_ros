#!/bin/bash

#
# START: CONFIGURATION OPTIONS
#

# The below two paths should point to the data set root and ROS package
package_source=~/catkin_ws/src/a2d2_to_ros
data_root=~/data/a2d2-preview

# This should point to the particular sensor fusion data set being converted
sensor_data=camera_lidar/20180810_150607

bus_data_subdir_full=/bus
bus_data_subdir_preview=
# Set this depending on which data you're converting (preview or full)
bus_data_subdir=$bus_data_subdir_preview

#
# END: CONFIGURATION OPTIONS
#

data_source="$data_root/$sensor_data"
sensor_locations=(cam_front_center cam_front_left cam_front_right cam_rear_center cam_side_left cam_side_right)

# Convert bus signal data
rosrun a2d2_to_ros sensor_fusion_bus_signals --sensor-config-json-path $data_root --sensor-config-schema-path $package_source/schemas/sensor_config.schema --bus-signal-json-path $data_source$bus_data_subdir --bus-signal-schema-path $package_source/schemas/sensor_fusion_bus_signal.schema || exit 1

# Convert sensor data
for location in "${sensor_locations[@]}"
do
  camera_data="$data_source/camera/$location"
  rosrun a2d2_to_ros sensor_fusion_camera --camera-data-path $camera_data --frame-info-schema-path $package_source/schemas/sensor_fusion_camera_frame.schema --sensor-config-path $data_root --sensor-config-schema-path $package_source/schemas/sensor_config.schema || exit 1

  lidar_data="$data_source/lidar/$location"
  rosrun a2d2_to_ros sensor_fusion_lidar --lidar-data-path $lidar_data --camera-data-path $camera_data --frame-info-schema-path $package_source/schemas/sensor_fusion_camera_frame.schema || exit 1
done
