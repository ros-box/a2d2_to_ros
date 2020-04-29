# A2D2 to ROS

Utilities for converting A2D2 data sets to ROS bags.

## Conventions

* Bus signal data is given the frame "bus".

## Notes

* Bus signal data is published to individual topics rather than collected into a single message. The reason for this is that much of the bus signal data is not time aligned, so it would be awkward to collect the various fields into an individual message.
