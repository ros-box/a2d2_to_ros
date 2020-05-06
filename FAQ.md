# FAQ for interpreting the A2D2 data set

This document contains a set of questions that might be unclear after examining readme file data at this page:

https://www.a2d2.audi/a2d2/en/download.html

And the technical description of the data set at this page:

https://arxiv.org/abs/2004.06320

Answers, where available, are given directly below the questions. If a question has no answer below it, then it remains an open question.

## General

1. Are all timestamps Unix Epoch time with microsecond units? (Mentar says they are, but the tutorial says they are TAI)
1. What is the precision of real-valued data (e.g., 32-bit, 64-bit, etc.)?
1. Are the labeled data a subset of the unlabeled data, or are they different sets?

## Frame conventions

1. For the reference frame ***g*** (described in [Section 3.1](https://arxiv.org/pdf/2004.06320.pdf)):
    1. Is the position of the origin fixed with respect to the vehicle?
    1. Is the z-axis is aligned to the gravity vector?
    1. Is the x-axis is aligned to vehicle heading?

## Sensor fusion bus signal data

1. What are the `distance_pulse_*` fields and what do the values represent?
1. What are the `latitude_direction` and `longitude_direction` fields and what do the values represent?
1. What are the conventions for the `steering_angle_calculated` values (i.e., what are min and max, and what is centered)?
1. What are the conventions for the `*_sign` fields (e.g., does it follow [std::signbit](https://www.cplusplus.com/reference/cmath/signbit/) conventions)?
1. Is `vehicle_speed` allowed to be negative? If not, how is driving in reverse indicated, or is it guaranteed that the vehicle never drives in reverse?
1. What is the convention for `accelerator_pedal` percent values (i.e., is 0 or 100 fully depressed)?

## Sensor fusion lidar data

1. What are the following lidar fields:
    1. `pcloud_attr.rectime`?
    1. `pcloud_attr.boundary`?
    1. `pcloud_attr.valid`?
1. Is `pcloud_attr.boundary` always 0 or 1?
1. What are the units for `pcloud_attr.reflectance`? Why are the values integral?
1. Are the `pcloud_attr.rectime` and `pcloud_attr.timestamp` intentionally `int64` and not `uint64`?
1. Are the points in the point cloud ordered in any way? If so, what is the ordering?
1. Are `pcloud_attr.depth` and `pcloud_attr.distance` strictly non-negative?
1. Are `pcloud_attr.row` and `pcloud_attr.col` supposed to be non-negative? At least `pcloud_attr.col` has negative values in the Munich sensor fusion data set.
1. Are the points motion compensated already?

## Semantic segmentation bus signal data

1. What is the `driving_direction` field and what do the values represent?
1. What do the values for the `gear` field represent?
1. Are `steering_angle` and `steering_angle_sign` ground truth with respect to `steering_angle_calculated` and `steering_angle_calculated_sign`?
