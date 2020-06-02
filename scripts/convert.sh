#!/bin/bash

#
# START: CONFIGURATION OPTIONS
#

# The below two paths should point to the data set root and ROS package
package_source=~/catkin_ws/src/a2d2_to_ros
data_root=~/data/a2d2

# This should point to the particular sensor fusion data set being converted
sensor_data=camera_lidar/20190401_145936

# Duration (in integer seconds) to record into a single bag file
split_duration=7

#
# END: CONFIGURATION OPTIONS
#

bus_data_subdir=/bus
data_source="$data_root/$sensor_data"
sensor_locations=(cam_front_center cam_front_left cam_front_right cam_rear_center cam_side_left cam_side_right)

# approximate duration (in integer seconds) of the data set
data_set_duration=746

# earliest time in the data set (in TAI microseconds) for which all sensor modalities provide data
record_start_time=1554121595035037

# Convert bus signal data
start_time=0
while [ $start_time -lt $data_set_duration ]
do
  end_time=$(( $start_time + $split_duration ))
  sub_dir="timespan_${start_time}s_${end_time}s"
  if [ ! -d "$sub_dir" ]; then
    mkdir $sub_dir
  fi

  rosrun a2d2_to_ros sensor_fusion_bus_signals --sensor-config-json-path $data_root --sensor-config-schema-path $package_source/schemas/sensor_config.schema --bus-signal-json-path $data_source$bus_data_subdir --bus-signal-schema-path $package_source/schemas/sensor_fusion_bus_signal.schema --min-time-offset $start_time --duration $split_duration --output-path $sub_dir --include-clock-topic true --start-time $record_start_time || exit 1

  start_time=$end_time
done


# Convert sensor data
for location in "${sensor_locations[@]}"
do
  start_time=0
  camera_data="$data_source/camera/$location"
  while [ $start_time -lt $data_set_duration ]
  do
    end_time=$(( $start_time + $split_duration ))
    sub_dir="timespan_${start_time}s_${end_time}s"
    if [ ! -d "$sub_dir" ]; then
      mkdir $sub_dir
    fi

    rosrun a2d2_to_ros sensor_fusion_camera --camera-data-path $camera_data --frame-info-schema-path $package_source/schemas/sensor_fusion_camera_frame.schema --sensor-config-path $data_root --sensor-config-schema-path $package_source/schemas/sensor_config.schema --min-time-offset $start_time --duration $split_duration --output-path $sub_dir --include-clock-topic false  --start-time $record_start_time || exit 1

    start_time=$end_time
  done


  lidar_data="$data_source/lidar/$location"
  start_time=0
  camera_data="$data_source/camera/$location"
  while [ $start_time -lt $data_set_duration ]
  do
    end_time=$(( $start_time + $split_duration ))
    sub_dir="timespan_${start_time}s_${end_time}s"
    if [ ! -d "$sub_dir" ]; then
      mkdir $sub_dir
    fi

    rosrun a2d2_to_ros sensor_fusion_lidar --lidar-data-path $lidar_data --camera-data-path $camera_data --frame-info-schema-path $package_source/schemas/sensor_fusion_camera_frame.schema --min-time-offset $start_time --duration $split_duration --output-path $sub_dir --include-clock-topic false  --start-time $record_start_time || exit 1

    start_time=$end_time
  done

done
