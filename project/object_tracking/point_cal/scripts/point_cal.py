#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from time import sleep
from image_geometry import PinholeCameraModel
import struct
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from nav_msgs.msg import Path
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from tf2_ros.buffer import Buffer
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class ref_calculate(Node):

    def __init__(self) :
        """
        Initialize the node, open serial port
        """        
        # Init node
        super().__init__('point_cal')
        # parameters
        self.declare_parameter('radius', 1.5)
        self.radius = self.get_parameter('radius').get_parameter_value().double_value

        self.declare_parameter('circle_num', 51)
        self.circle_num = self.get_parameter('circle_num').get_parameter_value().integer_value

        self.declare_parameter('topic_person', 'person_array')
        self.topic_person = self.get_parameter('topic_person').get_parameter_value().string_value

        self.declare_parameter('base_link_frame', 'base_link')
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value
       
        self.declare_parameter('camera_frame', 'camera_link')
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value


        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.person = []

        self.subscription_camera_left = self.create_subscription(
            Float32MultiArray,
            self.topic_person,
            self.listener_callback_object,
            1)
        
        # Publisher for MarkerArray
        self.pub_maker = self.create_publisher(MarkerArray, 'predect_circle', 10)

        
    def listener_callback_object(self, msg):
        point = self.send_tf(msg, self.camera_frame, self.base_link_frame)
        if len(point) == 2:
            self.person = [round(point[1], 4), round(point[0], 4)]
            print(self.person)

            th =  math.atan2(self.person[1], self.person[0])
            theta = np.linspace(th + math.pi/2, th + math.pi*3/2, self.circle_num)
            x = self.person[0] + self.radius * np.cos(theta)
            y = self.person[1] + self.radius * np.sin(theta)

            self.publish_predect_circle(x, y)


    def send_tf(self,msg, before_frame, after_frame):
        point = []
        x_data = msg.data[0::3]
        y_data = msg.data[1::3] 
        # print(x_data)
        num_object = len(x_data)
        for i in range(num_object):
            point_stamped = PoseStamped()
            point_stamped.pose.position.x = x_data[i]
            point_stamped.pose.position.y = y_data[i]
            point_stamped.header.frame_id = before_frame
            try:
                t = self.tf_buffer.transform(point_stamped, after_frame)
                point = [t.pose.position.x, t.pose.position.y]
            except:
                print('error')
                pass    
        return point
    
    def publish_predect_circle(self,x,y):
        markerArray = MarkerArray()
        num_object = len(x)
        for i in range(num_object):
            marker = Marker()
            marker.header.frame_id = self.base_link_frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.id = i
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = float(y[i])
            marker.pose.position.y = float(x[i])
            marker.pose.position.z = 0.2
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            markerArray.markers.append(marker)
        self.pub_maker.publish(markerArray)


def start():
    rclpy.init()
    ref = ref_calculate()
    try:
        rclpy.spin(ref)
        ref.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Closing ref calculate...")
    
   

if __name__ == '__main__':
    start()