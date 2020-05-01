# A2D2 to ROS

Utilities for converting A2D2 data sets to ROS bags.

## Use

The idea is that there is an executubale for each sensor modality: camera, lidar, and bus. Bag files are generated for these modalities independently and then merged (if desired) using standard ROS tools.

## Conventions

* Each field in the JSON file corresponds to four topics in the generated bag: one topic publishes the original value, one topic publishes the original units, one topic publishes the value in ROS units, and the last publishes the timestamp of the data (as a header message). This convention was adopted instead of defining a custom message so that the rosbag can be used without any dependency beyond stock ROS.
* Bus signal data is given the frame "bus".
* The message time in the bag file is the same as the timestamp in the header message.
* Options are not provided for changing the topic names; use ROS remapping functions for that.

## Notes

* Validate a json against the schema: `jsonschema -i [JSON_FILE] schemas/sensor_fusion_bus_signal.schema`
* Bus signal data is published to individual topics rather than collected into a single message. The reason for this is that much of the bus signal data is not time aligned, so it would be awkward to collect the various fields into an individual message.
