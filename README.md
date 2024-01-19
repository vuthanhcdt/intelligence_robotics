# Genbot - Generative AI Robot for [Networked robotic systems laboratory](https://sites.google.com/site/yenchenliuncku)
These packages made by [Thanh](https://sites.google.com/view/vuthanhcdt/home) and Hsin-Hui from [Networked robotic systems laboratory](https://sites.google.com/site/yenchenliuncku). If you use any packages from this repository, please cite this repository and our name.

## Overview
The project is constructed using Agilex's Scout Mini Omni platform, which are outlined as follows:
```
genbot/
├── genbot                          // Original packages
│   ├── scout_ros2                  // Original packages for Genbot
|   |   ├──scout_base               // Base for Genbot
|   |   ├──scout_msgs               // Messages for Genbot
|   |   ├──scout_simulation         // Simulation for Genbot
|   |   ├──velodyne_gazebo_plugins  // Gazebo plugins for Velodyne
|   ├── uwb                         // UWB DWM1001 for Genbot
│   ├── ugv_sdk                     // Protocol to transmit data with Genbot
├── thanh_pks                       // Some packages made by Thanh
├── hsin_hui_pkgs                   // Some packages made by Hsin-Hui
└── README.md
```

## Install Dependent ROS Packages
The project has undergone extensive testing on Ubuntu 22.04 (ROS Humble), making it strongly advisable to utilize Ubuntu 22.04 for optimal compatibility. To begin, open your terminal and execute the following commands:
```
sudo apt-get install ros-humble-joy ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard ros-humble-laser-proc \
  ros-humble-urdf ros-humble-xacro \
  ros-humble-compressed-image-transport ros-humble-rqt\
  ros-humble-interactive-markers \
  ros-humble-slam-toolbox\
  ros-humble-rqt ros-humble-rqt-common-plugins\
  ros-humble-gazebo-ros\
  ros-humble-robot-localization\
  ros-humble-realsense2-camera\
  ros-humble-realsense2-description\
  build-essential git cmake libasio-dev\
  ros-humble-tf2-geometry-msgs\
  ros-humble-eigen-stl-containers\
  ros-humble-ament-cmake-clang-format\
  ros-humble-nmea-msgs\
  ros-humble-mavros\
  ros-humble-navigation2\
  ros-humble-nav2-bringup\
  ros-humble-gazebo-ros-pkgs\
  ros-humble-bondcpp\
  ros-humble-ompl\
  ros-humble-turtlebot3-gazebo\
  ros-humble-pcl-ros\
  ros-humble-sensor-msgs-py\
  ros-humble-tf2-tools\
  gazebo\
  ros-humble-gazebo-ros-pkgs\
  ros-humble-ros-core\
  ros-humble-geometry2\
  xterm
```

## Creating a new package
If you wish to contribute to this project by creating a new package, please refrain from making any alterations to the existing repository. Instead, kindly establish a new folder under your name and develop your package within this designated folder. After creating your package, ensure you update the project overview and [README](README.md) file accordingly.

## ROS for Genbot
These commands enable the utilization of each function developed in Genbot with ``ROS_ID=150``.
### Install Genbot Packages
Open the new terminal and do these commands:
```
mkdir -p ~/genbot_ws/src
cd ~/genbot_ws/src/
git clone https://github.com/vuthanhcdt/genbot.git
cd ~/genbot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
echo "source ~/genbot_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=150" >> ~/.bashrc
```
### Bringup with Genbot

```
ros2 launch scout_base scout_mini_omni_base.launch.py
```
###  Control the robot with a keyboard teleoperation node

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### UWB
```
ros2 launch uwb uwb.launch.py 
```
Install pykalman to use Kalman filter for the position
```
pip3 install pyserial
pip3 install pykalman
```
### Gazebo with Agribot
Warehouse environment without the human:
```
ros2 launch scout_simulation simulation.launch.py 
```
Empty environment with the human:
```
ros2 launch scout_simulation simulation_empty_actor.launch.py 
```
To create a new environment, place it in the ``scout_simulation/worlds`` directory. If you desire to run simulations at a faster rate than real-time, you can modify the ``real_time_factor`` and ``real_time_update_rate`` parameters in the world file accordingly.

> **Note**  
> If you switch to a new operating system, ensure that all settings are properly installed for Genbot, please follow the [installation tutorials](https://github.com/vuthanhcdt/genbot/blob/humble/initial_setup.md) for each device.


## TODO
- [ ] ...
- [ ] ...
- [ ] ...
- [ ] ...
- [ ] ...

## References
1. https://sites.google.com/site/yenchenliuncku
