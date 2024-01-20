# Deep Learning for Object Detection and Object Following
This package contains essential commands to understand the basic principles of applying deep learning to object detection and tracking using YOLOv8.

If you want to try in the simulator, make sure Gazebo is enabled by running the following command:
```
ros2 launch scout_simulation object_tracking.launch.py
```
## 1. Object detection
The robot uses the YOLOv8 model for object recognition. If you wish to change the model, navigate to the ``object_detection/scripts/weights`` directory and rename the model file accordingly. To train for the custom model, please refer by [this tutorial](../train_model_yolov8).

```
ros2 launch object_detection object_detection.launch.launch.py #for real robot
ros2 launch object_detection object_detection_gazebo.launch.launch.py #for gazebo
```
## 2. Object tracking
After successfully detecting objects, a navigation framework is employed based on the relative positions of the objects for tracking. The robot will halt if it cannot detect any objects along its path.
```
ros2 launch object_tracking object_tracking.launch.py 
```