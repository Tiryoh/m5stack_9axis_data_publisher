# m5stack_9axis_data_publisher

## Requirements

[AtsushiSaito/rt_usb_9axis_sensor](https://github.com/AtsushiSaito/rt_usb_9axis_sensor)

## Installation

1. Flash m5stack_9axis_data_publisher.ino to your M5Stack.
2. Install and setup ROS on your PC.

## Usage

Connect M5Stack to PC with USB-C cable, and run:

```
roslaunch rt_usb_9axis_sensor rt_usb_9axis_sensor.launch
```

[![demo1](./docs/img/m5stack_9axis_data_publisher-demo1.gif)](https://gyazo.com/bdf5c1ed2495e31a79371ea9a408f942)

## Known Errors

* "MPU9250 I AM 0 I should be 71"
  * http://forum.m5stack.com/topic/281/doesn-t-work-sample-sketch-mp9250basicahrs-on-m5stack-gray/6
