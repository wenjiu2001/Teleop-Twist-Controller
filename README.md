# Teleop-Twist-Controller

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC_BY--NC--SA_4.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Ubuntu:Focal](https://img.shields.io/badge/Ubuntu-Focal-brightgreen)](https://releases.ubuntu.com/focal/)
[![ROS:Noetic](https://img.shields.io/badge/ROS-Noetic-blue)](https://wiki.ros.org/noetic/Installation/Ubuntu)

"teleop_twist_controller" is a joystick control package utilized for the movement of ROS robots.

## Requirements

- Joystick
   ```
   sudo apt-get install joystick jstest-gtk
   ```
- Pygame
   ```
   python -m pip install -U pygame==2.5.2 --user
   ```
- Xbox Controller
   ```
   sudo apt-add-repository -y ppa:rael-gc/ubuntu-xboxdrv
   ```
   ```
   sudo apt-get update
   ```
   ```
   sudo apt-get install xboxdrv
   ```
   ```
   sudo sh -c "echo blacklist xpad >> /etc/modprobe.d/blacklist.conf"
   ```
   ```
   sudo modprobe xpad
   ```

## Install and Build

1. Navigating to the "src" directory within your catkin workspace :
   ```
   cd ~/catkin_ws/src
   ```
2. Clone teleop_twist_controller package for github :
   ```
   git clone https://github.com/wenjiu2001/Teleop-Twist-Controller.git teleop_twist_controller
   ```
3. Build teleop_twist_controller package :
   ```
   cd ~/catkin_ws && catkin_make
   ```
4. Package environment setup :
   ```
   source ./devel/setup.sh
   ```

## How to Use

Adjustments can be made by modifying the following parameters:

| Parameter name | Data Type | detail                                                       |
| -------------- | ------- | ------------------------------------------------------------ |
| device_number| int | Set the controller to be used. <br/>default: `5` |
| speed        | float | Set the maximum and minimum linear velocities. <br/>default: `0.26` |
| turn         | float | Set the maximum and minimum angular velocities. <br/>default: `1.82` |
| cmd_vel_topic| string | Set the name of the topic to be published. <br/>default: `/cmd_vel` |
| repeat_rate  | float | Set the frequency of continuous `cmd_vel_topic` updates. <br/>default: `0.0` |
| stamped      | bool  | Set the message format to be `TwistStamped`. <br/>default: `False` |
| frame_id     | string | Set the coordinate system of the message, exclusively applicable to the `TwistStamped` message format. <br/>default: `base_link` |

- Activate joystick control :
   ```
   roslaunch teleop_twist_controller teleop_twist_controller.launch
   ```
   
## References

- [Ubuntu install of ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- [teleop_twist_keyboard](https://wiki.ros.org/teleop_twist_keyboard)
- [pygame](https://www.pygame.org/news)
