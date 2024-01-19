# IRobot - Intelligence Robotics Course

Welcome to the iRobot Intelligence Robotics Course offered by the [Networked Robotic Systems Laboratory (NRSL)](https://sites.google.com/site/yenchenliuncku). This repository contains essential packages developed by [NRSL](https://sites.google.com/site/yenchenliuncku) to support various aspects of the course. If you choose to utilize any packages from this repository, we kindly request you to cite this repository and acknowledge [NRSL](https://sites.google.com/site/yenchenliuncku) in your work.


## Overview
The project is constructed using Agilex's Scout Mini Omni platform, which are outlined as follows:
```
irobot/
├── irobot                          // Original packages
│   ├── scout_ros2                  // Original packages for Irobot
|   |   ├──scout_base               // Base for Irobot
|   |   ├──scout_msgs               // Messages for Irobot
|   |   ├──scout_simulation         // Simulation for Irobot
|   |   ├──velodyne_gazebo_plugins  // Gazebo plugins for Velodyne
│   |   ├──ugv_sdk                  // Protocol to transmit data with 
|   ├── project                     // Project packages for Irobot
|   |   ├──train_model_yolov8       // Package for training YOLOv8 model
|   |   ├──autonoumous_navigation   // Package for autonomous navigation
|   |   ├──object_tracking          // Package for object tracking
Irobot
├── student_pks                     // PStudent packages - Place your packages here with your name to test your programs.
└── README.md
```
Please ensure that your packages are named in lowercase and do not contain any special characters. Place them in the ``student_pkgs`` directory.
## Course Resources
Here are some useful links for the course:
- [W3Schools - Python Tutorial](https://www.w3schools.com/python/): Comprehensive Python tutorials covering basics to advanced concepts.
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html): Official documentation for ROS 2, offering guides and essential resources for learners.
- [Understanding ROS 2 topics](https://docs.ros.org/en/crystal/Tutorials/Topics/Understanding-ROS2-Topics.html): Explore the fundamentals of ROS 2 topics.
- [ROS 2 Publishers and Subscribers Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html): Learn about Publishers and Subscribers in ROS 2 through this tutorial.
- [Yolov8](https://docs.ultralytics.com/usage/python/#train): Explore Yolov8 training documentation for detailed information.

Ensure to explore these links for detailed and helpful resources throughout your learning and program development. Happy coding!


## Install Dependent ROS Packages
The source code is designed to be used on Ubuntu 22.04 (ROS Humble), making it highly recommended to utilize Ubuntu 22.04 for optimal compatibility. To get started, open your terminal and execute the following commands to install the dependent packages for ROS2:
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
## Install Irobot Workspace
Open the new terminal and do these commands:
```
mkdir -p ~/irobot_ws/src
cd ~/irobot_ws/src/
https://github.com/vuthanhcdt/intelligence_robotics.git
cd ~/irobot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
echo "source ~/irobot_ws/install/setup.bash" >> ~/.bashrc
```
Commands for iRobot Usage

- Launch the simulation in a warehouse environment with Gazebo.
```
ros2 launch scout_simulation simulation.launch.py
```
- Run the keyboard teleoperation node to control the robot using the keyboard.

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Launch the iRobot base to bring up the robot.

```
ros2 launch scout_base scout_mini_omni_base.launch.py
```
## Project material
Please refer to the README files for each package in the ``project``directory.
1. [Object recognition using Deep Learning with YOLOv8](project/train_model_yolov8/README.md).
2. 



## References
1. https://sites.google.com/site/yenchenliuncku
