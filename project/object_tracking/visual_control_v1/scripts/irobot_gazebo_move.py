#!/usr/bin/env python3
from numba import cuda
import numpy as np
import math
import numba
import copy
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, PoseStamped,Point, Pose2D
from sensor_msgs.msg import Imu
from std_msgs.msg import Int64, Float64

# Information about your GPU
gpu = cuda.get_current_device()
max_threads_per_block = gpu.MAX_THREADS_PER_BLOCK
max_square_block_dim = (int(gpu.MAX_BLOCK_DIM_X**0.5), int(gpu.MAX_BLOCK_DIM_X**0.5))
max_blocks = gpu.MAX_GRID_DIM_X  # 2147483647
rec_max_control_rollouts = int(1e6) # Though theoretically limited by max_blocks on GPU
rec_min_control_rollouts = 100

class Config:

    """ Configurations that are typically fixed throughout execution. """

    def __init__(self,
                T = 5.0,
                dt = 0.1,
                num_vel_rollouts = 10,
                num_ang_vel_rollouts = 10,
                seed = 1
                ):
        assert T > 0
        assert dt > 0
        assert T > dt

        self.seed = seed
        self.T = T
        self.dt = dt
        self.num_steps = int(T/dt)

        # Number of control rollouts are currently limited by the number of blocks
        if (num_vel_rollouts * num_ang_vel_rollouts) > rec_max_control_rollouts:
            self.num_control_rollouts_vel = num_vel_rollouts
            self.num_control_rollouts_ang_vel = int(np.sqrt(rec_max_control_rollouts))
            self.num_control_rollouts = int(np.sqrt(rec_max_control_rollouts))
            print("DWA Config: Clip num_control_rollouts to be recommended max number of {}. (Max={})".format(
            rec_max_control_rollouts, max_blocks))
        else:
            self.num_control_rollouts_vel = num_vel_rollouts
            self.num_control_rollouts_ang_vel = num_ang_vel_rollouts
            self.num_control_rollouts = self.num_control_rollouts_vel * self.num_control_rollouts_ang_vel

        self.num_vis_state_rollouts = self.num_control_rollouts

class DWA_Numba(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.T = cfg.T
        self.dt = cfg.dt
        self.num_steps = cfg.num_steps
        self.num_control_rollouts_vel = cfg.num_control_rollouts_vel
        self.num_control_rollouts_ang_vel = cfg.num_control_rollouts_ang_vel
        self.num_control_rollouts = cfg.num_control_rollouts

        self.u_seq0 = np.zeros((self.num_control_rollouts, 2), dtype=np.float32)
        self.params_set = False

        self.init_device_vars_before_solving()

    def init_device_vars_before_solving(self):
       self.u_option_numba = cuda.to_device(self.u_seq0)
       self.rng_states_numba = cuda.device_array((self.num_control_rollouts, self.num_steps+1, 3), dtype=np.float32)
       self.costs_numba = cuda.device_array((self.num_control_rollouts), dtype=np.float32)
       self.costs_angle_numba = cuda.device_array((self.num_control_rollouts), dtype=np.float32)
       self.costs_vel_numba = cuda.device_array((self.num_control_rollouts), dtype=np.float32)
       self.costmap = np.array((40, 40), dtype=np.int8)
       self.dis_obstable_numba = cuda.device_array((self.num_control_rollouts, self.num_steps+1), dtype=np.float32)
       self.costs_obstable_numba = cuda.device_array((self.num_control_rollouts), dtype=np.float32)
       self.warning_obstable_numba = cuda.device_array((self.num_control_rollouts), dtype=np.float32)

       self.device_var_initialized = True

    def setup(self, params):
        self.set_params(params)

    def set_params(self, params):
        self.params = copy.deepcopy(params)
        self.params_set = True

    def shift_and_update(self, u_cur, costmap, cur_goal):
        self.u_cur_numba = cuda.to_device(u_cur.astype(np.float32))
        self.u_prev_numba = cuda.to_device(u_cur.astype(np.float32))
        self.costmap = costmap.copy()
        self.goal_numba = cuda.to_device(cur_goal.astype(np.float32))

    def check_solve_conditions(self):
        # print("check_solve_conditions")
        if not self.params_set:
            print("DWA parameters are not set. Cannot solve")
            return False
        if not self.device_var_initialized:
            print("Device variables not initialized. Cannot solve.")
            return False
        return True
    
    def solve(self):
        """Entry point for different algoritims"""
        if not self.check_solve_conditions():
            print("DWA solve condition not met. Cannot solve. Return")
            return
        
        return self.solve_with_nominal_dynamics()
    
    def move_dwa_tasks_vars_to_device(self):
        robot_threshold_numba = np.float32(self.params['robot_threshold'])
        obstacle_tolerance_numba = np.float32(self.params['obstacle_tolerance'])
        map_resolution = np.float32(self.params['map_resolution'])
        map_origin_x = np.float32(self.params['map_origin_x'])
        map_origin_y = np.float32(self.params['map_origin_y'])
        weight_angle_numba = np.float32(self.params['weight_angle'])
        weight_vel_numba = np.float32(self.params['weight_vel'])
        weight_obs_numba = np.float32(self.params['weight_obs'])
        score_obstacle_numba = np.float32(self.params['score_obstacle'])
        max_acc = np.float32(self.params['max_acc'])
        max_ang_acc = np.float32(self.params['max_ang_acc'])
        lim_max_vel = np.float32(self.params['lim_max_vel'])
        lim_min_vel = np.float32(self.params['lim_min_vel'])
        lim_max_ang_vel = np.float32(self.params['lim_max_ang_vel'])
        return robot_threshold_numba, obstacle_tolerance_numba, map_resolution, map_origin_x, map_origin_y, \
                weight_angle_numba, weight_vel_numba, weight_obs_numba, score_obstacle_numba, \
                max_acc, max_ang_acc, lim_max_vel, lim_min_vel, lim_max_ang_vel

    def return2zero(self):
        self.u_option_numba[:] = 0.0
        self.rng_states_numba[:] = 0.0
        self.costs_numba[:] = 0.0
        self.costs_angle_numba[:] = 0.0
        self.costs_vel_numba[:] = 0.0
        self.dis_obstable_numba[:] = 0.0
        self.costs_obstable_numba[:] = 0.0
        self.warning_obstable_numba[:] = int(-1)
        self.u_cur = [0.0, 0.0]

    def get_state_rollout(self):
        self.max_index = np.argmax(self.costs_numba)
        rng_states_host = self.rng_states_numba.copy_to_host()
        self.opt_states_numba = rng_states_host[self.max_index]
        return self.rng_states_numba.copy_to_host(), self.opt_states_numba
    
    def get_cost_info(self):
        return self.costs_numba.copy_to_host(), \
                self.costs_angle_numba.copy_to_host(), self.costs_vel_numba.copy_to_host(), self.costs_obstable_numba.copy_to_host(), \
                self.warning_obstable_numba.copy_to_host()
    
    def set_range(self, max_acc, max_ang_acc, lim_max_vel, lim_min_vel, lim_max_ang_vel):        
        range_ang_vel = self.dt * max_ang_acc
        min_ang_vel = max(self.u_prev_numba[1] - range_ang_vel, - lim_max_ang_vel)
        max_ang_vel = min(self.u_prev_numba[1] + range_ang_vel, lim_max_ang_vel)
        delta_ang_vel = (max_ang_vel - min_ang_vel) / (self.num_control_rollouts_ang_vel - 1)

        range_vel = self.dt * max_acc
        min_vel = max(self.u_prev_numba[0] - range_vel, lim_min_vel)
        max_vel = min(self.u_prev_numba[0] + range_vel, lim_max_vel)
        delta_vel = (max_vel - min_vel) / (self.num_control_rollouts_vel -1)
        return delta_vel, delta_ang_vel, min_vel,  max_vel, min_ang_vel, max_ang_vel
    
    def solve_with_nominal_dynamics(self):
        robot_threshold_numba, obstacle_tolerance_numba, map_resolution, map_origin_x, map_origin_y, \
            weight_angle_numba, weight_vel_numba, weight_obs_numba, score_obstacle_numba, \
            max_acc, max_ang_acc, lim_max_vel, lim_min_vel, lim_max_ang_vel = self.move_dwa_tasks_vars_to_device()
        
        self.return2zero()

        delta_vel, delta_ang_vel, \
                min_vel,  max_vel, min_ang_vel, max_ang_vel = self.set_range(max_acc, max_ang_acc, lim_max_vel, lim_min_vel, lim_max_ang_vel)

        self.make_u[self.num_control_rollouts, 1](self.num_control_rollouts_ang_vel,
                                                    min_vel, max_vel,
                                                    min_ang_vel, max_ang_vel,
                                                    delta_vel, delta_ang_vel,

                                                    self.u_option_numba)
        
        self.make_path[self.num_control_rollouts, 1](self.u_option_numba,
                                                     self.dt,
                                                     self.num_steps,
                                                     
                                                     self.rng_states_numba)
        
        self._calc_heading_angle_score[self.num_control_rollouts, 1](self.goal_numba,
                                                                     self.rng_states_numba,
                                                                     
                                                                     self.costs_angle_numba)   

        self._calc_heading_vel_score[self.num_control_rollouts, 1](self.u_option_numba,
                                                                   
                                                                   self.costs_vel_numba)

        if len(self.costmap) > 1 :
            self._calc_obstacles_costmap[self.num_control_rollouts, self.num_steps + 1](self.costmap,
                                                                                        map_resolution,
                                                                                        map_origin_x,
                                                                                        map_origin_y,
                                                                                        self.rng_states_numba,
                                                                                        
                                                                                        self.dis_obstable_numba)
            
            self._calc_obstacles_score[self.num_control_rollouts, 1](self.dis_obstable_numba,
                                                                     self.costs_obstable_numba,
                                                                     self.warning_obstable_numba)

        self._calc_score[self.num_control_rollouts, 1](self.costs_angle_numba,
                                                       self.costs_vel_numba,
                                                       self.costs_obstable_numba,
                                                       weight_angle_numba,
                                                       weight_vel_numba,
                                                       weight_obs_numba,
                                                       
                                                       self.costs_numba)
        
        choose = 3
        top_two = np.argpartition(self.costs_numba.copy_to_host(), -choose)[-choose:]  # Get indices of two largest elements
        for i in range(choose):
            self.u_cur += self.u_option_numba.copy_to_host()[top_two[i]]/ choose

        return self.u_cur
        

        
    @staticmethod
    @cuda.jit(fastmath=True)
    def make_u(num_control_rollouts_ang_vel,
               min_vel, max_vel,
               min_ang_vel, max_ang_vel,
               delta_vel, delta_ang_vel, u_option):
        bid = cuda.blockIdx.x
        u_option[bid, 0] = min_vel + int(bid / (num_control_rollouts_ang_vel)) * delta_vel
        u_option[bid, 1] = min_ang_vel + int(bid % (num_control_rollouts_ang_vel)) * delta_ang_vel

    @staticmethod
    @cuda.jit(fastmath=True)
    def make_path(u_option_numba,
                  dt,
                  num_steps,

                  rng_states_numba):
        bid = cuda.blockIdx.x
        for i in range(num_steps + 1):
            if i == 0:
                rng_states_numba[bid, i, 0] = 0.0
                rng_states_numba[bid, i, 1] = 0.0
                rng_states_numba[bid, i, 2] = 0.0
            else:
                rng_states_numba[bid, i, 0] = rng_states_numba[bid, i-1, 0] + u_option_numba[bid, 0] * np.cos(rng_states_numba[bid, i-1, 2]) * dt
                rng_states_numba[bid, i, 1] = rng_states_numba[bid, i-1, 1] + u_option_numba[bid, 0] * np.sin(rng_states_numba[bid, i-1, 2]) * dt
                rng_states_numba[bid, i, 2] = rng_states_numba[bid, i-1, 2] + u_option_numba[bid, 1] * dt

    @staticmethod
    @cuda.jit(fastmath=True)
    def _calc_heading_angle_score(goal_numba,
                                  rng_states_numba,
                                  
                                  costs_angle_numba):
        bid = cuda.blockIdx.x
        ang = np.arctan2((goal_numba[1] - rng_states_numba[bid, -1, 1]), (goal_numba[0] - rng_states_numba[bid, -1, 0])) - rng_states_numba[bid, -1, 2]
        
        if ang > np.pi:
            while ang > np.pi:
                ang -= 2 * np.pi
        elif ang < -np.pi:
            while ang < -np.pi:
                ang += 2 * np.pi
        costs_angle_numba[bid] = np.pi - abs(ang)

    @staticmethod
    @cuda.jit(fastmath=True)
    def _calc_heading_vel_score(u_option_numba,
                                
                                costs_vel_numba):
        bid = cuda.blockIdx.x
        costs_vel_numba[bid] = u_option_numba[bid, 0]

    @staticmethod
    @cuda.jit(fastmath=True)
    def _calc_obstacles_costmap(costmap,
                                map_resolution,
                                map_origin_x,
                                map_origin_y,
                                rng_states_numba,
                                
                                dis_obstable_numba):
        bid = cuda.blockIdx.x
        tid = cuda.threadIdx.x
        score = 0
        if tid < len(rng_states_numba[bid]):
            x_tmp = int((rng_states_numba[bid, tid, 1] - map_origin_x) / map_resolution)
            y_tmp = int((rng_states_numba[bid, tid, 0] - map_origin_y) / map_resolution)
            score = costmap[x_tmp-1, y_tmp+1] +      costmap[x_tmp, y_tmp+1] + costmap[x_tmp+1, y_tmp+1]+ \
                    costmap[x_tmp-1, y_tmp+0] + 1.5 *costmap[x_tmp, y_tmp+0] + costmap[x_tmp+1, y_tmp+0]+ \
                    costmap[x_tmp-1, y_tmp-1] +      costmap[x_tmp, y_tmp-1] + costmap[x_tmp+1, y_tmp-1]

            dis_obstable_numba[bid, tid] = -score

    @staticmethod
    @cuda.jit(fastmath=True)
    def _calc_obstacles_score(dis_obstable_numba,
                              cost_obstable_numba,
                              warning_obstable_numba):
        bid = cuda.blockIdx.x
        cost_obstable_numba[bid] = min(dis_obstable_numba[bid])
        # for i in range(len(dis_obstable_numba[bid])):
        #     cost_obstable_numba[bid] += dis_obstable_numba[bid, i]
        # print(bid, cost_obstable_numba[bid])
        if cost_obstable_numba[bid] < -150.0:
            warning_obstable_numba[bid] = 1.0

    @staticmethod
    @cuda.jit(fastmath=True)
    def _calc_score(costs_angle_numba,
                    costs_vel_numba,
                    costs_obstable_numba,
                    weight_angle_numba,
                    weight_vel_numba,
                    weight_obs_numba,
                    
                    costs_nmba):
        bid = cuda.blockIdx.x
        costs_nmba[bid] = costs_angle_numba[bid] * weight_angle_numba + costs_vel_numba[bid] * weight_vel_numba + \
                            costs_obstable_numba[bid] * weight_obs_numba 


class LocalPlanningNode(Node):
    def __init__(self):
        super().__init__('DWA_planner')
        self.qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=1)
        self.pre_vel = np.array([0.0, 0.0])
        self.goal = np.array([0.0, 0.0, 0.0])

        self.T = self.declare_parameter('T',5.0).get_parameter_value().double_value
        self.samplingtime = self.declare_parameter('samplingtime',0.1).get_parameter_value().double_value
        self.num_vel_rollouts = self.declare_parameter('num_vel_rollouts',10).get_parameter_value().integer_value
        self.num_ang_vel_rollouts = self.declare_parameter('num_ang_vel_rollouts',200).get_parameter_value().integer_value
        self.robot_threshold = self.declare_parameter('robot_threshold',0.35).get_parameter_value().double_value
        self.dis_safe_obstacle = self.declare_parameter('dis_safe_obstacle',0.45).get_parameter_value().double_value
        self.map_resolution = self.declare_parameter('map_resolution',0.25).get_parameter_value().double_value
        self.map_origin_x = self.declare_parameter('map_origin_x',-5.0).get_parameter_value().double_value
        self.map_origin_y = self.declare_parameter('map_origin_y',-5.0).get_parameter_value().double_value
        self.weight_angle = self.declare_parameter('weight_angle',0.025).get_parameter_value().double_value
        self.weight_vel = self.declare_parameter('weight_vel',0.1).get_parameter_value().double_value
        self.weight_obs = self.declare_parameter('weight_obs',0.1).get_parameter_value().double_value
        self.score_obstacle = self.declare_parameter('score_obstacle',2.0).get_parameter_value().double_value
        self.max_acc = self.declare_parameter('max_acc',3.0).get_parameter_value().double_value
        self.max_ang_acc = self.declare_parameter('max_ang_acc',10.0).get_parameter_value().double_value       
        self.lim_max_vel = self.declare_parameter('lim_max_vel',0.7).get_parameter_value().double_value
        self.lim_min_vel = self.declare_parameter('lim_min_vel',0.1).get_parameter_value().double_value
        self.lim_max_ang_vel = self.declare_parameter('lim_max_ang_vel',0.78539).get_parameter_value().double_value
        self.goal_shift = self.declare_parameter('goal_shift',0.5).get_parameter_value().double_value

        self.get_goal = False

        self.sub = self.create_subscription(Pose2D, "goal_point", self.listener_callback_goal, self.qos)
        self.subscription = self.create_subscription(OccupancyGrid, 'occupancy_grid_topic', self.listener_callback_occupancyGrid, qos_profile_sensor_data)

        self.cfg = Config(
                    T = self.T,
                    dt = self.samplingtime,
                    num_vel_rollouts = self.num_vel_rollouts,
                    num_ang_vel_rollouts = self.num_ang_vel_rollouts,
                    seed = 1)
        
        self.dwa_params = dict(robot_threshold = self.robot_threshold,
                               obstacle_tolerance = self.dis_safe_obstacle,
                               map_resolution = self.map_resolution,
                               map_origin_x = self.map_origin_x,
                               map_origin_y = self.map_origin_y,
                               weight_angle = self.weight_angle,
                               weight_vel = self.weight_vel,
                               weight_obs = self.weight_obs,
                               score_obstacle = self.score_obstacle,
                               max_acc = self.max_acc,
                               max_ang_acc = self.max_ang_acc,
                               lim_max_vel = self.lim_max_vel,
                               lim_min_vel = self.lim_min_vel,
                               lim_max_ang_vel = self.lim_max_ang_vel)
        

        self.dwa_planner = DWA_Numba(self.cfg)
        self.dwa_planner.setup(self.dwa_params)

        self.timer = self.create_timer(0.01, self.timer_callback)  # Timer for periodic execution

        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 3)
        self.local_path_publisher = self.create_publisher(Path, '/local_path', 10)
        self.sampled_path_publisher = self.create_publisher(MarkerArray, '/sampled_path', 10)

    def listener_callback_goal(self, msg):
        self.goal[0] = msg.x
        self.goal[1] = msg.y
        self.goal[2] = msg.theta
        self.get_goal = True
        self.robot_position = False

    def listener_callback_occupancyGrid(self, msg):
        self.map_size_x = int(2.0* abs(self.map_origin_x) / self.map_resolution)
        self.map_size_y = int(2.0* abs(self.map_origin_y) / self.map_resolution)
        assert self.map_size_x == int(np.sqrt(len(msg.data))), "Size wrong !"
        assert self.map_size_y == int(np.sqrt(len(msg.data))), "Size wrong !"
        self.costmap = np.array(msg.data).reshape(int(np.sqrt(len(msg.data))), int(np.sqrt(len(msg.data))))
        
    def publish_path(self,x,y,publisher,base):    
            path_msg = Path()
            path_msg.header.frame_id = base # Set the appropriate frame ID # odom base_link
            length=len(x)
            for idx in range(length):
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = float(x[idx])
                pose_stamped.pose.position.y = float(y[idx])
                path_msg.poses.append(pose_stamped)
            publisher.publish(path_msg)

    def sampled_path(self, x,y, warning_obstable, base):
        marker_array = MarkerArray()        
        id_path = 0
        num_path=len(x[0])
        for idx in range(num_path):
            marker = Marker()
            marker.header.frame_id = base # odom base_link
            # marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'robot_paths'
            marker.id = id_path
            id_path += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.01
            if warning_obstable[idx] > 0.0:
                marker.color.r = 0.75
                marker.color.g = 0.75
                marker.color.b = 0.75
                marker.color.a = 1.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            length_path=len(x)
            for i in range(length_path):
                point = Point()
                point.x = float(x[i][idx])
                point.y = float(y[i][idx])
                marker.points.append(point)
            marker_array.markers.append(marker)
        self.sampled_path_publisher.publish(marker_array)

    def timer_callback(self):
        twist = Twist()
        self.current_useq = self.pre_vel
        
        if self.get_goal == True:
            distance = math.hypot(self.goal[0], self.goal[1])
            self.robot_position=True
            if distance < self.goal_shift:
                self.robot_position = False
        
        if self.robot_position==True:

            self.dwa_planner.shift_and_update(self.current_useq, self.costmap, self.goal)
            useq = self.dwa_planner.solve()

            twist.linear.x = float(useq[0])
            twist.angular.z = float(useq[1])

            self.rollout_states_vis, self.opt_states_vis = self.dwa_planner.get_state_rollout()
            cost, cost_head, cost_vel, cost_obs, warning_obstable = self.dwa_planner.get_cost_info()
            opt_states_vis_x = [point[0] for point in self.opt_states_vis]
            opt_states_vis_y = [point[1] for point in self.opt_states_vis]
            self.publish_path(opt_states_vis_x,opt_states_vis_y,self.local_path_publisher,"base_link")
            self.sampled_path(self.rollout_states_vis[:,:,0].T,self.rollout_states_vis[:,:,1].T, warning_obstable,"base_link")

            self.pre_vel = np.array(useq.tolist())


        self.pub_vel.publish(twist)
        # self.get_goal = False











def start():

    rclpy.init()
    planning = LocalPlanningNode()
    try:
        rclpy.spin(planning)
        planning.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Closing local_planning...")

if __name__ == '__main__':
    start()

