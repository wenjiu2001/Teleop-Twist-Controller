# Teleop-Twist-Controller

"teleop_twist_controller" is a joystick control package utilized for the movement of ROS robots.

## Environment

- Ubuntu 20.04
- ROS Noetic
- Python 3.8.10

## Requirements

- Joystick
   ```
   sudo apt-get install joystick jstest-gtk
   ```
- Pygame
   ```
   python -m pip install -U pygame==2.5.2 --user
   ```
- Xbox Controllers
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

`count` for selecting which controller to use, with a default value of 4
`speed` to adjust the maximum linear velocity, defaulting to 0.26
`turn` to regulate the maximum turning speed, with a standard setting of 1.82
`repeat_rate` for continuous cmd_vel updates, with a standard setting of 0.0

- Run Teleop Twist Controller :
   ```
   rosrun teleop_twist_controller teleop_twist_controller
   ```
- Launch Teleop Twist Controller :
   ```
   roslaunch teleop_twist_controller teleop_twist_controller.launch
   ```
   
## References

- Ubuntu install of ROS Noetic (https://wiki.ros.org/noetic/Installation/Ubuntu)
- Install Gazebo using Ubuntu packages (https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
- TurtleBot3 Simulation (https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- teleop_twist_keyboard (https://wiki.ros.org/teleop_twist_keyboard)
- pygame (https://www.pygame.org/news)
