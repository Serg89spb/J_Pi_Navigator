#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/serg89spb/ros2_ws/install/setup.bash

# Используем полный путь к ros2 и лаунчеру
/opt/ros/jazzy/bin/ros2 launch /home/serg89spb/robot_launch_nav.py
