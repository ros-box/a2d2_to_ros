# FAQ for interpreting the A2D2 data set

This document contains a set of questions that might be unclear after examining readme file data at this page:

https://www.a2d2.audi/a2d2/en/download.html

And the technical description of the data set at this page:

https://arxiv.org/abs/2004.06320

Answers, where available, are given directly below the questions and have come from discussions with the [A2D2 Team](https://www.a2d2.audi/a2d2/en/team.html). If a question has no answer below it, then it remains an open question.

## General

1. **Do all subsets in the unlabeled data set span the same time?**
    * ?
1. **What convention do the timestamps follow?**
    * *All timestamps encode microseconds since Epoch begin. Bus signal timestamps are in [UTC](https://en.wikipedia.org/wiki/Coordinated_Universal_Time), all others are [TAI](https://en.wikipedia.org/wiki/International_Atomic_Time)*
1. **What is the precision of real-valued data (e.g., single precision, double precision, etc.)?**
    * *None of the data in the data set was recorded with greater than single precision. (For lidar data, note that numpy stores the point data with double precision, but the data itself is only generated with single precision.)*
1. **Are the labeled data a subset of the unlabeled data, or are they different sets?**
    * *The object detection dataset is a subset of the semantic segmentation dataset. The raw sensor fusion data (Gaimersheim, Ingolstadt, and Munich) is a different set.*

## Conventions

1. **Are all units in the `cams_lidars.json` file SI?**
    * ?
1. **Are coordinate systems right handed?**
    * ?
1. **In the bus signal data, what are the `roll_angle` and `pitch_angle` conventions?**
    * ?
1. **What are the conventions for the reference frame ***g*** (described in [Section 3.1](https://arxiv.org/pdf/2004.06320.pdf))?**
    * *The frame ***g*** is fixed with respect to the chassis of the vehicle, and all sensor poses are static with respect to it.*
    * *In addition, there is a `wheels` frame that, at rest, is coincident with ***g***. In motion, ***g*** can roll and pitch (but not yaw or translate) with respect to `wheels`. In the bus signal data, the `roll_angle` and `pitch_angle` values describe these offsets.*

## Sensor configuration

1. **The tutorial states that `tstamp_delay` "specifies a known delay in microseconds between actual camera frame times"; what does that mean? Are these delays accounted for in the frame timestamps?**
    * *This is baked for the semantic segmentation and object detection datasets but not for the raw datasets. You can use this delay to register lidar/camera timestamps. Lidar comes before camera, and therefore tstamp_delay correction is optionally needed here.*
1. **What are the `CorrectedImagePort` fields in `cams_lidars.json` for the `side_left` and `rear_center` cameras? Why are they `null`?**
    * *These fields are not used and can be ignored.*
1. **What are the `view_` fields in the camera sections of `cams_lidars.json`?**
    * *These fields are not used and can be ignored.*

## Sensor fusion bus signal data

1. **There is no `yaw_angle` field; is there supposed to be?**
    * *No. See the reference frame ***g*** topic in the 'Conventions' section.*
1. **What are the `distance_pulse_*` fields and what do the values represent?**
    * *Distance pulse data comes from pulse counter at the wheels. Counters generally run forward, even when driving backwards. If the sensor is reset due to error or failure, the value is always, without exception, to send 1 times the Init value 1021. If the value 1021 is not present in the data set, then there were resets due to error or failure.*
1. **What are the `latitude_direction` and `longitude_direction` fields and what do the values represent?**
    * *latitude_direction: North/South Hemisphere (0 north / 1 south), longitude_direction: East/West Hemisphere (0 east, 1 west)*
1. **What are the conventions for the `steering_angle_calculated` values (i.e., what are min and max, and what is centered)?**
    * *Steering angle with straight-line flow correction from the position of the steering motor (0 = centered, stepsize = 0.15, min = 0, max = 614.25, init = 614.10, error = 614.25).*
1. **What are the conventions for the `*_sign` fields?**
    * *0 = left/positive, 1 = right/negative*
1. **Is `vehicle_speed` allowed to be negative?**
    * *No, the vehicle_speed is always between 0 and 655.35.*
1. **What is the convention for `accelerator_pedal` percent values (i.e., is 0 or 100 fully depressed)?**
    * *100 = fully depressed*

## Sensor fusion lidar data

1. **Are the points in the point cloud ordered in any way? If so, what is the ordering?**
    * ?
1. **What is the `pcloud_attr.rectime` lidar data field?**
    * *Rectime is the recording time when this data point was recorded. It is in TAI (i.e. no leap seconds) time zone and not UTC. The same is true for the camera timestamps in the json files. However, the bus data is in UTC and therefore there’s a delta of 37s when matching/registering data from camera/lidar to bus.*
1. **What is the `pcloud_attr.boundary` lidar data field?**
    * *Boundary flag is true if the data point is coming from bordering lidar channel, probably ring 0 and ring 15 in the data set (recorded with VLP16).*
1. **What is the `pcloud_attr.valid` lidar data field?**
    * *Valid is only true for points that need to be considered. There are some points in the xy plane near the sensor that need not be considered. That is, if valid != true, don’t consider this point.*
1. **What are the units for `pcloud_attr.reflectance`?**
    * *See Velodyne VLP16 manual Sec 6.1*
1. **Are `pcloud_attr.depth` and `pcloud_attr.distance` strictly non-negative?**
    * *Yes.*
1. **Are `pcloud_attr.row` and `pcloud_attr.col` supposed to be non-negative? What is the convention to convert to integer pixel coordinates?**
    * *They may be negative but then they would fall outside the image, and thus not really usable. You may choose whatever pixel conversion convention makes sense for the application you have in mind. Typically they are simply rounded to the nearest integer.*
1. **Are the lidar points in the raw sensor fusion data set motion compensated already?**
    * *No.*

## Semantic segmentation bus signal data

1. **What is the `driving_direction` field and what do the values represent?**
    * *The signal driving_direction indicates reverse driving (0 = direction not definined, 1 = foreward, 2 = reverse, 3 = standstill). Note that in the raw sensor fusion data set there is no reverse driving so the 'driving_direction' is not needed and not included in the bus signal data.*
1. **What do the values for the `gear` field represent?**
    * *0 = Gear N, 1 = Gear 1, 2 = Gear 2, ..., 8 = Gear 8, 9 = Automatic P, 10 = Automatic forward S, 11 = Automatic forward D, 12 = Intermediate position, 13 = reverse gear, 14 = gear not defined, 15 = Error*
1. **Are `steering_angle` and `steering_angle_sign` ground truth with respect to `steering_angle_calculated` and `steering_angle_calculated_sign`?**
    * *steering_angle is the position / shift from the middle measured on the steering link (it's no angle -> bad naming) and steering_angle_calculated is the steering wheel angle.*

## Discrepancies with online tutorial

These items have to do with variations between the data and the [tutorial](https://www.a2d2.audi/a2d2/en/tutorial.html).

### Questions

1. **The JSON info file associated with each camera image has the additional fields `image_zoom` and `pcld_view` that are not listed in the tutorial; what are they?**
    * ?
1. **What is the purpose of the `get_axes_of_a_view` method? Why would the `x-axis` and `y-axis` members of the `view` objects not already be orthonormal?**
    * *The axes might already be orthogonal; the method is just being pedantic.*

### Notes

The tutorial lists the lidar data fields as:

```
['azimuth', 'row', 'lidar_id', 'depth', 'reflectance', 'col', 'points', 'timestamp', 'distance']
```

However, in the data set they are:

```
['pcloud_points', 'pcloud_attr.rectime', 'pcloud_attr.azimuth', 'pcloud_attr.reflectance', 'pcloud_attr.boundary', 'pcloud_attr.lidar_id', 'pcloud_attr.timestamp', 'pcloud_attr.valid', 'pcloud_attr.row', 'pcloud_attr.col', 'pcloud_attr.distance', 'pcloud_attr.depth']
```
