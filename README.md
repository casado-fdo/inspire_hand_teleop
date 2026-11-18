# Inspire Hand Teleoperation

ROS package for teleoperating the Inspire-Robots RH56DFX dexterous hand with a Leap Motion controller.

## Overview
`leap_to_inspire_hand.py` maps Leap Motion finger tracking to the six hand joints, with fist and pinch gestures and smoothing. `teleop_inspire_hand_controller.py` sends the commands to the hand, applies force feedback and grasping, and moves the hand to its ready pose on an inactivity timeout.

## Prerequisites
- ROS Noetic
- [leap-motion-ros](https://github.com/casado-fdo/leap-motion-ros), publishing `/leapmotion/hands/{left,right}`
- [inspire_hand_ros](https://github.com/mperialCollegeLondon/inspire_hand_ros), providing the hand driver

## Installation
Clone the repository into a catkin workspace and build:
```bash
cd ~/catkin_ws/src
git clone https://github.com/casado-fdo/inspire_hand_teleop.git
cd ..
catkin_make
```

## Usage
Start the Leap Motion and Inspire hand drivers, then launch the teleoperation for one hand:
```bash
roslaunch inspire_hand_teleop leap_teleop.launch hand_side:=right
```
Use `hand_side:=left` for the left hand. The other launch arguments are `leap_ns` (default `/leapmotion`), `hand_ns` (default `/inspire_hand`), `rate` (default 100 Hz, controller only), `param_file` (default [`config/default.yaml`](config/default.yaml)) and `namespace`. For a dual-hand setup, include the launch file once per hand with different `hand_side`, `hand_ns` and `namespace`, as `blueberry_driver/launch/full_leap_teleop.launch` does.

## ROS Interface
| Topic | Type | Description |
|---|---|---|
| `/leapmotion/hands/{left,right}` | `leap_motion_controller/Hand` | Leap Motion input (subscribed) |
| `/inspire_hand/leap_teleop/hand_angles` | `std_msgs/Int32MultiArray` | Joint angles generated from Leap data |
| `/inspire_hand/set_angles` | `sensor_msgs/JointState` | Commands sent to the hand driver; `-1` leaves a joint unchanged |
| `/inspire_hand/hand_state` | `std_msgs/Int32MultiArray` | Six joint positions followed by six force readings |
| `/inspire_hand/set_clear_error` | `inspire_hand/set_clear_error` | Service used to clear hand errors |

The topic namespace follows `hand_ns`. The launch file sets `teleop_source` to `leap_teleop` by default. With `imitation_mode:=inference`, or any mode other than `demo_teleop`, it becomes `lerobot` and the controller reads `/inspire_hand/lerobot/hand_angles` instead.

## Configuration
Control parameters (angle limits, gesture thresholds, smoothing, force threshold, inactivity timeout) live in [`config/default.yaml`](config/default.yaml).

## License
This project is licensed under the MIT License. See [`LICENSE`](LICENSE) for details.

## Acknowledgments
Developed at the Personal Robotics Laboratory, Imperial College London.
