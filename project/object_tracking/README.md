# Deep Learning for Object Detection and Object Following
This package includes essential commands to understand the basic principles of applying deep learning to object detection and tracking by using Yolov8.

If you want to try in the simulator, make sure Gazebo is enabled by running the following command:
```
ros2 launch scout_simulation object_tracking.launch.py
```
## Object detection
The robot uses the YOLOv8 model for object recognition. If you wish to change the model, navigate to the ``object_detection/scripts/weights`` directory and rename the model file accordingly. To train for the custom model, please refer by [this tutorial]((project/train_model_yolov8)).

```
ros2 launch object_detection object_detection.launch.launch.py #for real robot
ros2 launch object_detection object_detection_gazebo.launch.launch.py #for gazebo
```
