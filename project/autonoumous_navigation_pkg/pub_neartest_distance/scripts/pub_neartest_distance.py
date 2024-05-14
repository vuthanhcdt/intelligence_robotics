#!/usr/bin/env python3
import rclpy, math, tf2_ros
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from time import sleep
from image_geometry import PinholeCameraModel
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Pose2D, Pose
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class ref_calculate(Node):

    def __init__(self) :
        """
        Initialize the node, open serial port
        """        
        # Init node
        super().__init__('pub_neartest_distance')

        self.qos = QoSProfile(depth=1)
        self.qos_scan = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=1)

        # parameters
        self.laser_max_dis = self.declare_parameter('laser_max_dis', 5.0).get_parameter_value().double_value
        self.base_link_frame = self.declare_parameter('base_link_frame', 'base_link').get_parameter_value().string_value
        self.base_scan_frame = self.declare_parameter('base_scan_frame', 'base_scan').get_parameter_value().string_value

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, self.qos_scan)
        
        # Publisher for MarkerArray
        self.pub_maker = self.create_publisher(MarkerArray, 'predect_circle', 10)
        self.goal_pub = self.create_publisher(Pose2D, 'goal_point', self.qos)
        self.oval_pub = self.create_publisher(MarkerArray, "oval_point", self.qos)

    def check_angle(self, theta:float):
        while(abs(theta) > math.pi):
            theta = theta - self.check_sign(theta)*math.pi
        return theta
    
    def check_sign(self,n:float):
        return -1 if n < 0 else 1
    
    def transform_laser(self, laser_msg:LaserScan):
        min_ang = laser_msg.angle_min
        increment = laser_msg.angle_increment
        self.obstacle = np.array([])
        for i, value in enumerate(laser_msg.ranges):
            angle = self.check_angle(i * increment + min_ang)
            if value < self.laser_max_dis:
                x = math.cos(angle) * value
                y = math.sin(angle) * value
                point_stamped = PoseStamped()
                point_stamped.pose.position.x = float(x)
                point_stamped.pose.position.y = float(y)
                point_stamped.pose.position.z = 1.0
                point_stamped.header.frame_id = self.base_scan_frame
                try:
                    t = self.tf_buffer.transform(point_stamped, self.base_link_frame)
                    point_new = Point()
                    point_new.x = t.pose.position.x
                    point_new.y = t.pose.position.y
                    point_new.z = t.pose.position.z
                    self.obstacle.append([point_new.x,point_new.y])
                    if self.get_laser == False:
                        self.get_laser = True
                except:
                    # print('transform error!')
                    pass 

    def scan_cb(self, msg:LaserScan):
        self.transform_laser(msg)
        min = 1e10
        for i in range (np.size(self.obstacle)):
            dis = math.hypot(self.obstacle[i,0],self.obstacle[i,1])
            if dis < min:
                min = dis


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