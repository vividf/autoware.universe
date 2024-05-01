# object_velocity_splitter

This package contains a object filter module for [autoware_auto_perception_msgs/msg/DetectedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/DetectedObject.idl).
This package can split DetectedObjects into two messages by object's speed.

## Interface

### Input

- `~/input/objects` (`autoware_auto_perception_msgs/msg/DetectedObjects.msg`)
  - 3D detected objects

### Output

- `~/output/low_speed_objects` (`autoware_auto_perception_msgs/msg/DetectedObjects.msg`)
  - Objects with low speed
- `~/output/high_speed_objects` (`autoware_auto_perception_msgs/msg/DetectedObjects.msg`)
  - Objects with high speed

### Parameters

- `velocity_threshold` (double) [m/s]
  - Default parameter is 3.0

This parameter is velocity threshold to split objects
