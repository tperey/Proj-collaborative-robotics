#!/bin/bash

# Update package list and install prerequisites
sudo apt-get update && sudo apt-get install -yq lsb-release

# Create workspace directory
mkdir -p dev_ws/src && cd dev_ws/src

# Remove ROS default sources list and initialize rosdep
sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update --include-eol-distros

# Install ROS package
sudo apt-get install -yq ros-galactic-rtabmap-ros

# Clone necessary repositories
git clone https://github.com/Interbotix/interbotix_ros_core.git -b galactic --recurse-submodules
git clone https://github.com/Interbotix/interbotix_ros_rovers.git -b galactic
git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git -b galactic
git clone https://github.com/ros-planning/moveit_visual_tools.git -b ros2

# Install dependencies
cd ..
rosdep install --from-paths src --ignore-src --rosdistro galactic -r -y

# Build the workspace
cd dev_ws
source /opt/ros/galactic/setup.bash
colcon build

# Install additional ROS2 packages
sudo apt install -y ros-galactic-irobot-create-description
pip install modern_robotics

sudo apt install -y \
ros-$ROS_DISTRO-gazebo-ros2-control \
ros-$ROS_DISTRO-gazebo-ros2-control-demos \
ros-$ROS_DISTRO-effort-controllers \
ros-$ROS_DISTRO-joint-state-broadcaster



INSTALL_PATH=dev_ws
BASE_TYPE=create3
echo -e "export INTERBOTIX_XSLOCOBOT_BASE_TYPE=${BASE_TYPE}"  >> ~/.bashrc
echo -e "export INTERBOTIX_WS=${INSTALL_PATH}"                >> ~/.bashrc
echo -e "source /usr/share/gazebo/setup.sh"                >> ~/.bashrc


echo "source dev_ws/install/setup.bash" >> ~/.bashrc

