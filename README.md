# m5stack_9axis_data_publisher

## Requirements

[AtsushiSaito/rt_usb_9axis_sensor](https://github.com/AtsushiSaito/rt_usb_9axis_sensor)

## Installation

1. Flash m5stack_9axis_data_publisher.ino to your M5Stack.
2. Install and setup ROS on your PC.

## Usage

```
roslaunch rt_usb_9axis_sensor rt_usb_9axis_sensor.launch
```

## Known Errors

* "MPU9250 I AM 0 I should be 71"
  * http://forum.m5stack.com/topic/281/doesn-t-work-sample-sketch-mp9250basicahrs-on-m5stack-gray/6
