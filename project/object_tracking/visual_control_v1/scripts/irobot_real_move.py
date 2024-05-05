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
from geometry_msgs.msg import Twist, PoseStamped,Point
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

        # Initialize reuseable device variables
        self.u_cur = [0.0, 0.0]
        self.u_cur_numba = None
        self.u_prev_numba = None
        self.costs_numba = None
        self.rng_states_numba = None
        self.opt_states_numba = None
        self.max_index = None

        # Other task specific params
        self.device_var_initialized = False
        self.spr_numba = None
        self.obs_pos_numba = None
        self.obs_r_numba = None

        # reset
        self.u_seq0 = np.zeros((self.num_control_rollouts, 2), dtype=np.float32)
        self.cost_size = np.zeros(self.num_control_rollouts, dtype=np.float32)
        self.current_state = np.zeros((3), dtype=np.float32)
        self.params = None
        self.params_set = False
        self.u_prev_numba = None
        self.goal_num = None
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
       self.dis_spraying_numba = cuda.device_array((self.num_control_rollouts, self.num_steps+1), dtype=np.float32)
       self.dis_spraying_final_numba = cuda.device_array((self.num_control_rollouts), dtype=np.float32)
       self.costs_spraying_numba = cuda.to_device(self.cost_size) 
       self.dis_spraying_sort_numba = cuda.to_device(self.cost_size) 

       self.device_var_initialized = True

    def move_dwa_tasks_vars_to_device(self):
        # print("move_dwa_tasks_vars_to_device")
        robot_threshold_numba = np.float32(self.params['robot_threshold'])
        goal_tolerance_numba = np.float32(self.params['goal_tolerance'])
        obstacle_tolerance_numba = np.float32(self.params['obstacle_tolerance'])
        map_resolution = np.float32(self.params['map_resolution'])
        map_origin_x = np.float32(self.params['map_origin_x'])
        map_origin_y = np.float32(self.params['map_origin_y'])
        weight_angle_row_numba = np.float32(self.params['weight_angle_row'])
        weight_angle_turn_numba = np.float32(self.params['weight_angle_turn'])
        weight_angle_goal_numba = np.float32(self.params['weight_angle_goal'])
        weight_vel_numba = np.float32(self.params['weight_vel'])
        weight_obs_numba = np.float32(self.params['weight_obs'])
        weight_spr_row_numba = np.float32(self.params['weight_spr_row'])
        weight_spr_turn_numba = np.float32(self.params['weight_spr_turn'])
        weight_spr_goal_numba = np.float32(self.params['weight_spr_goal'])
        score_obstacle_numba = np.float32(self.params['score_obstacle'])
        max_acc = np.float32(self.params['max_acc'])
        max_ang_acc = np.float32(self.params['max_ang_acc'])
        lim_max_vel = np.float32(self.params['lim_max_vel'])
        lim_min_vel = np.float32(self.params['lim_min_vel'])
        lim_max_ang_vel = np.float32(self.params['lim_max_ang_vel'])
        return robot_threshold_numba ,goal_tolerance_numba ,obstacle_tolerance_numba, map_resolution, map_origin_x, map_origin_y,\
                weight_angle_row_numba ,weight_angle_turn_numba,weight_angle_goal_numba, \
                weight_vel_numba,weight_obs_numba, \
                weight_spr_row_numba ,weight_spr_turn_numba,weight_spr_goal_numba, score_obstacle_numba,\
                max_acc, max_ang_acc, lim_max_vel, lim_min_vel, lim_max_ang_vel
    
    def return2zero(self):
        self.u_option_numba[:] = 0.0
        self.rng_states_numba[:] = 0.0
        self.costs_numba[:] = 0.0
        self.costs_angle_numba[:] = 0.0
        self.costs_vel_numba[:] = 0.0
        self.dis_obstable_numba[:] = 0.0
        self.costs_obstable_numba[:] = 0.0
        self.dis_spraying_numba[:] = 0.0
        self.dis_spraying_final_numba[:] = 0.0
        self.costs_spraying_numba[:] = 0.0
        self.dis_spraying_sort_numba[:] = 0.0
        self.warning_obstable_numba[:] = int(-1)
        self.u_cur = [0.0, 0.0]

    def solve_with_nominal_dynamics(self):
        robot_threshold_numba ,goal_tolerance_numba ,obstacle_tolerance_numba, map_resolution, map_origin_x, map_origin_y, \
                weight_angle_row_numba ,weight_angle_turn_numba,weight_angle_goal_numba, \
                weight_vel_numba,weight_obs_numba, \
                weight_spr_row_numba ,weight_spr_turn_numba,weight_spr_goal_numba, score_obstacle_numba,\
                max_acc, max_ang_acc, lim_max_vel, lim_min_vel, lim_max_ang_vel = self.move_dwa_tasks_vars_to_device()
        
        self.return2zero()

        delta_vel, delta_ang_vel, \
                min_vel,  max_vel, min_ang_vel, max_ang_vel = self.set_range(max_acc, max_ang_acc, lim_max_vel, lim_min_vel, lim_max_ang_vel)
        
        self.make_u[self.num_control_rollouts, 1](self.num_control_rollouts_vel,
                                                     self.num_control_rollouts_ang_vel,
                                                     min_vel,
                                                     max_vel,
                                                     min_ang_vel,
                                                     max_ang_vel,
                                                     delta_vel,
                                                     delta_ang_vel,
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

        obs_num = len(self.obs_pos_numba)
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

        # if len(self.obs_pos_numba) > 1 :
        #     self._calc_obstacles_dis[self.num_control_rollouts, self.num_steps + 1](score_obstacle_numba,
        #                                                                             robot_threshold_numba,
        #                                                                             obstacle_tolerance_numba,
        #                                                                             obs_num,
        #                                                                             self.obs_pos_numba,
        #                                                                             self.obs_r_numba,
        #                                                                             self.rng_states_numba,
                                                                                    
        #                                                                             self.dis_obstable_numba)
        #     self._calc_obstacles_score[self.num_control_rollouts, 1](self.dis_obstable_numba,
        #                                                              self.costs_obstable_numba,
        #                                                              self.warning_obstable_numba)

        if len(self.spr_numba) > 1 :
            self._calc_spraying_dis[self.num_control_rollouts, self.num_steps+1](self.rng_states_numba,
                                                                self.spr_numba,
                                                                self.dis_spraying_numba)
            
            self._calc_spraying_sum[self.num_control_rollouts, 1](self.dis_spraying_numba,
                                                                  self.dis_spraying_final_numba)

            # max_dis = max(self.dis_spraying_final_numba)
            # min_dis = min(self.dis_spraying_final_numba)
            self._calc_spraying_minmax[self.num_control_rollouts, 1](self.dis_spraying_final_numba, 
                                                                     self.dis_spraying_sort_numba)
            min_dis  = self.dis_spraying_sort_numba.copy_to_host()[0]
            max_dis  = self.dis_spraying_sort_numba.copy_to_host()[-1]

            self._calc_spraying_score[self.num_control_rollouts, 1](max_dis,min_dis,
                                                                    self.dis_spraying_final_numba,
                                                                    self.costs_spraying_numba)


        dis = math.sqrt((self.goal_numba[0])**2 + (self.goal_numba[1])**2)
        if dis <= goal_tolerance_numba:
            weight_angle = weight_angle_goal_numba
            weight_spr = weight_spr_goal_numba
            weight_vel = weight_vel_numba / 10.0
        elif self.goal_num % 2 == 1:
            weight_angle = weight_angle_row_numba
            weight_spr = weight_spr_row_numba
            weight_vel = weight_vel_numba
        else:
            weight_angle = weight_angle_turn_numba
            weight_spr = weight_spr_turn_numba
            weight_vel = weight_vel_numba

        self._calc_score[self.num_control_rollouts, 1](self.costs_angle_numba,
                                                       self.costs_vel_numba,
                                                       self.costs_obstable_numba,
                                                       self.costs_spraying_numba,
                                                       weight_angle,
                                                       weight_vel,
                                                       weight_obs_numba,
                                                       weight_spr,
                                                       
                                                       self.costs_numba)
        choose = 3
        top_two = np.argpartition(self.costs_numba.copy_to_host(), -choose)[-choose:]  # Get indices of two largest elements
        for i in range(choose):
            self.u_cur += self.u_option_numba.copy_to_host()[top_two[i]]/ choose

        return self.u_cur
    
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
    
    def setup(self, params):
        self.set_params(params)

    def set_params(self, params):
        self.params = copy.deepcopy(params)
        self.params_set = True
    
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
    
    def shift_and_update(self, new_x0, u_cur, costmap, obstacle_positions, obstacle_radius, spraying_line,  cur_goal, goal_num, num_shifts=1):
        self.current_state = new_x0.copy()
        self.shift_optimal_control_sequence(u_cur, num_shifts)
        self.costmap = costmap.copy()
        self.obs_pos_numba = cuda.to_device(obstacle_positions.astype(np.float32))
        self.obs_r_numba = cuda.to_device(obstacle_radius.astype(np.float32))
        self.spr_numba = cuda.to_device(spraying_line.astype(np.float32))

        diff_x = cur_goal[0] - new_x0[0]
        diff_y = cur_goal[1] - new_x0[1]
        matrix_tf = np.array([[ math.cos(new_x0[2]), math.sin(new_x0[2])], \
                              [-math.sin(new_x0[2]), math.cos(new_x0[2])]]) 
        matrix_diff = np.array([[diff_x],[diff_y]])
        g_tmp = np.dot(matrix_tf, matrix_diff)
        g_local = np.array([g_tmp[0][0], g_tmp[1][0]])
        self.goal_numba = cuda.to_device(g_local.astype(np.float32))
        self.goal_num = goal_num

    def shift_optimal_control_sequence(self, u_cur, num_shifts=1):
        self.u_cur_numba = cuda.to_device(u_cur.astype(np.float32))
        self.u_prev_numba = cuda.to_device(u_cur.astype(np.float32))

    def get_state_rollout(self):
        self.max_index = np.argmax(self.costs_numba)
        rng_states_host = self.rng_states_numba.copy_to_host()
        self.opt_states_numba = rng_states_host[self.max_index]
        return self.rng_states_numba.copy_to_host(), self.opt_states_numba
    
    def get_cost_info(self):
        return self.costs_numba.copy_to_host(), \
                self.costs_angle_numba.copy_to_host(), self.costs_vel_numba.copy_to_host(), self.costs_obstable_numba.copy_to_host(), self.costs_spraying_numba.copy_to_host(),\
                self.warning_obstable_numba.copy_to_host()
        
    @staticmethod
    @cuda.jit(fastmath=True)
    def make_u(num_control_rollouts_vel,
               num_control_rollouts_ang_vel,
               min_vel, max_vel,
               min_ang_vel, max_ang_vel,
               delta_vel, delta_ang_vel, u_option):
        bid = cuda.blockIdx.x
        # print(bid, int(bid / (num_control_rollouts_vel)))
        # print(bid, int(bid % (num_control_rollouts_ang_vel)))
        u_option[bid, 0] = min_vel + int(bid / (num_control_rollouts_ang_vel)) * delta_vel
        u_option[bid, 1] = min_ang_vel + int(bid % (num_control_rollouts_ang_vel)) * delta_ang_vel

    @staticmethod
    @cuda.jit(fastmath=True)
    def make_path(u_option_numba,
                  dt,
                  num_steps,

                  rng_states_numba):
        bid = cuda.blockIdx.x
        score = cuda.local.array(51, dtype=np.float32)
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
        # if abs(rng_states_numba[bid, -1, 2]) > 1.4:costs_angle_numba[bid] = costs_angle_numba[bid] * 0.5

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
        if tid < len(rng_states_numba[bid]) and len(costmap) > 1:
            x_tmp = int((rng_states_numba[bid, tid, 1] - map_origin_x) / map_resolution)
            y_tmp = int((rng_states_numba[bid, tid, 0] - map_origin_y) / map_resolution)
            score = costmap[x_tmp-1, y_tmp+1] +      costmap[x_tmp, y_tmp+1] + costmap[x_tmp+1, y_tmp+1]+ \
                    costmap[x_tmp-1, y_tmp+0] + 1.5 *costmap[x_tmp, y_tmp+0] + costmap[x_tmp+1, y_tmp+0]+ \
                    costmap[x_tmp-1, y_tmp-1] +      costmap[x_tmp, y_tmp-1] + costmap[x_tmp+1, y_tmp-1]

            dis_obstable_numba[bid, tid] = -score

    @staticmethod
    @cuda.jit(fastmath=True)
    def _calc_obstacles_dis(score_obstacle_numba,
                            robot_threshold_numba,
                            obstacle_tolerance_numba,
                            obs_num,
                            obs_pos_numba,
                            obs_r_numba,
                            rng_states_numba,
                            
                            dis_obstable_numba):
        bid = cuda.blockIdx.x
        tid = cuda.threadIdx.x
        score_obstacle_sqrd = score_obstacle_numba ** 2
        for i in range(obs_num):
            temp_dis_to_obs_sqrd = (rng_states_numba[bid, tid, 0] - obs_pos_numba[i,0]) ** 2 + (rng_states_numba[bid, tid, 1] - obs_pos_numba[i,1]) ** 2
            if math.sqrt(temp_dis_to_obs_sqrd) < (robot_threshold_numba + obstacle_tolerance_numba + obs_r_numba[i]):
                dis_obstable_numba[bid, tid] = -200 
                break
            score_obstacle_sqrd = min(score_obstacle_sqrd, temp_dis_to_obs_sqrd)
            dis_obstable_numba[bid, tid] = math.sqrt(score_obstacle_sqrd)

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
    def _calc_spraying_dis(rng_states_numba,
                        spr_numba,
                        dis_spraying_numba):
        bid = cuda.blockIdx.x
        tid = cuda.threadIdx.x
        if tid < len(spr_numba):
            dis_spraying_numba[bid, tid] = np.hypot(rng_states_numba[bid, tid, 0] - spr_numba[tid, 0],
                                                    rng_states_numba[bid, tid, 1] - spr_numba[tid, 1])
            
    @staticmethod
    @cuda.jit(fastmath=True)
    def _calc_spraying_sum(dis_spraying_numba,
                        dis_spraying_final_numba):
        bid = cuda.blockIdx.x
        tid = cuda.threadIdx.x
        for i in range(len(dis_spraying_numba[bid])):
            dis_spraying_final_numba[bid] += dis_spraying_numba[bid, i]

    @staticmethod
    @cuda.jit(fastmath=True)
    def _calc_spraying_minmax(dis_spraying_final_numba,
                              dis_spraying_sort_numba):
        bid = cuda.blockIdx.x
        tid = cuda.threadIdx.x
        idx = tid + bid * cuda.blockDim.x
        count = 0
        count_same = 0
        for i in range(len(dis_spraying_final_numba)):
            if dis_spraying_final_numba[i] < dis_spraying_final_numba[bid] and (i != bid):
                count += 1
            if dis_spraying_final_numba[i]  == dis_spraying_final_numba[bid] and i < bid:
                count_same += 1
        dis_spraying_sort_numba[count + count_same] = dis_spraying_final_numba[bid]

    @staticmethod
    @cuda.jit(fastmath=True)
    def _calc_spraying_score(max_dis,min_dis,
                             dis_spraying_numba,
                             costs_spraying_numba):
        bid = cuda.blockIdx.x
        costs_spraying_numba[bid] = (max_dis - dis_spraying_numba[bid]) / (max_dis - min_dis)

    @staticmethod
    @cuda.jit(fastmath=True)
    def _calc_score(costs_angle_numba,
                    costs_vel_numba,
                    costs_obstable_numba,
                    costs_spraying_numba,
                    weight_angle,
                    weight_vel_numba,
                    weight_obs_numba,
                    weight_spr,
                    
                    costs_nmba):
        bid = cuda.blockIdx.x
        costs_nmba[bid] = costs_angle_numba[bid] * weight_angle + costs_vel_numba[bid] * weight_vel_numba + \
                            costs_obstable_numba[bid] * weight_obs_numba + costs_spraying_numba[bid] * weight_spr

class LocalPlanningNode(Node):
    def __init__(self):
        super().__init__('DWA_planner')

        ### gazebo_env_straight_line_missing
        # self.goal = [(-0.5, -44.6), (-0.4915, 45.3809), (4.9098, 41.5258), (4.5685, -45.8360), (10.0696, -42.7917), (10.3097, 11.6328), (10.5172, 29.7851),
        #              (10.5974, 44.6257), (16.1017, 41.5989),(16.1240, 32.01735), (15.9977, 13.6516), (15.4607, -46.2922), (21.2043, -43.2922), (21.9674, 44.5989), 
        #              (27.8317, 41.4095), (27.3520, -13.1320), (27.1978, -33.8936), (27.1085, -46.3076), (32.3365, -42.6907),(32.9040, 46.4390),(37.7597, 43.4390), (37.3136, -46.6907)]
        
        ### gazebo_env_small_angle_bend_missing
        # self.goal = [(-0.5, -13.46), (1.7038, 47.1062), (6.5845, 41.5258), (4.6378, -16.4642), (10.0718, -12.3499), (10.2583, 25.2424), (10.4839, 36.2993), 
        #       (11.5977, 45.5258), (17.1690, 41.3848), (15.857830, 35.967346), (16.031263, 24.251932),  (15.6247, -17.1278), (21.5462, -13.1278), (22.8001, 45.5989), (28.2674, 41.0773), (25.4410, 3.6638), (25.6391, -9.0713),
        #       (27.4431, -17.2418), (32.8971, -12.5849), (33.6994, 45.0773), (38.8560, 40.9792), (38.1268, -15.5849)]
        
        ### gazebo_env_big_angle_bend_missing
        # self.goal = [(-5.5, -14.2), (-1.5, 30.55), (14.85, 31.96), (15.4939, 66.1750), (20.5609, 62.1750), (-1.25, 0.0), (-0.476, -8.659), (-0.4736, -18.5698), (4.6025, -13.1218),  (25.7678, 67.9176), (31.4866, 61.9608), (9.8164, -18.3702), (15.1848, -11.8136), (15.0756, 3.8088), (20.9764, 10.6456), (37.1179, 45.2225), (37.2428, 56.5818),
        #              (37.3811, 66.9608), (42.7464, 61.9097), (20.2128, -18.8212), (25.0650, -13.5689), (48.1679, 67.7944), (53.5399, 62.7944), (30.0269, -16.5689)] 
        
        ### gazebo_env_right_angle_bend_missing
        # self.goal = [(-0.4308, -29.2997), (0.003, 24.24), (2.003, 25.244), (45.7090, 26.1581), (40.7090, 20.8428), (4.515, 20.4837), (4.4413, 14.8317) , (4.5012, -33.2997), (9.3323, -27.2731), 
        #              (9.95, 11.255), (8.35, 14.17), (25.4274, 15.2608), (35.7132, 15.3730),(45.7192, 15.3688), (40.6411, 9.8398), (17.46, 9.7), (14.4347, 4.73), (14.4636, -1.3712), (14.5210, -10.4957),
        #              (14.3936, -32.2731), (19.6024, -27.0854), (45.7976, 4.3758), (40.4694, -0.6716), (24.6917, -30.0854), (29.8483, -27.0422), (45.4694, -6.1510), (40.4094, -11.6767), (34.9622, -30.0422)]

        ### indoor gps
        # self.goal = [(3.65, 2.86), (7.2, 0.0)]
            
        ### outdoor gps
        self.goal = [(-14.477, -29.832), (-41.851, -27.798), (-41.717, -33.429), (-16.552, -34.861), (-17.152, -40.404), (-42.578, -38.682), (-43.132, -44.217), (-16.355, -45.718)]
        # self.goal = [(-14.477, -29.832),(-35.634, -28.328)]

        self.goal_num = 1
        self.qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=1)
        self.costmap = np.array((40, 40), dtype=np.int8)
        self.obstacle_positions = np.array([])
        self.obstacle_radius = np.array([])
        self.spraying_line = np.array([])
        self.traj_paths = []
        self.pre_state=np.array([0.0, 0.0 , 0.0])
        self.pre_vel = np.array([0.0, 0.0])
        self.mileage = 0.0
                
        self.cal_mileage = False
        self.read_pos_data = False
        self.read_ang_data = False
        self.robot_position=False

        self.declare_parameter('print_data',False)
        self.print_data = self.get_parameter('print_data').get_parameter_value().bool_value

        self.declare_parameter('samplingtime',0.1)
        self.samplingtime = self.get_parameter('samplingtime').get_parameter_value().double_value

        self.declare_parameter('dis_change_goal',1.5)
        self.goal_shift = self.get_parameter('dis_change_goal').get_parameter_value().double_value

        self.declare_parameter('goal_tolerance',3.0)
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        self.declare_parameter('T',5.0)
        self.T = self.get_parameter('T').get_parameter_value().double_value

        self.declare_parameter('robot_threshold',0.35)
        self.robot_threshold = self.get_parameter('robot_threshold').get_parameter_value().double_value

        self.declare_parameter('dis_safe_obstacle',0.45)
        self.dis_safe_obstacle = self.get_parameter('dis_safe_obstacle').get_parameter_value().double_value

        self.declare_parameter('map_resolution',0.25)
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value

        self.declare_parameter('map_origin_x',-5.0)
        self.map_origin_x = self.get_parameter('map_origin_x').get_parameter_value().double_value

        self.declare_parameter('map_origin_y',-5.0)
        self.map_origin_y = self.get_parameter('map_origin_y').get_parameter_value().double_value

        self.declare_parameter('num_vel_rollouts',10)
        self.num_vel_rollouts = self.get_parameter('num_vel_rollouts').get_parameter_value().integer_value

        self.declare_parameter('num_ang_vel_rollouts',200)
        self.num_ang_vel_rollouts = self.get_parameter('num_ang_vel_rollouts').get_parameter_value().integer_value

        self.declare_parameter('weight_angle_row',0.025)
        self.weight_angle_row = self.get_parameter('weight_angle_row').get_parameter_value().double_value

        self.declare_parameter('weight_angle_turn',0.5)
        self.weight_angle_turn = self.get_parameter('weight_angle_turn').get_parameter_value().double_value

        self.declare_parameter('weight_angle_goal',0.6)
        self.weight_angle_goal = self.get_parameter('weight_angle_goal').get_parameter_value().double_value

        self.declare_parameter('weight_vel',0.1)
        self.weight_vel = self.get_parameter('weight_vel').get_parameter_value().double_value

        self.declare_parameter('weight_obs',0.1)
        self.weight_obs = self.get_parameter('weight_obs').get_parameter_value().double_value

        self.declare_parameter('weight_spr_row',0.9)
        self.weight_spr_row = self.get_parameter('weight_spr_row').get_parameter_value().double_value

        self.declare_parameter('weight_spr_turn',0.0)
        self.weight_spr_turn = self.get_parameter('weight_spr_turn').get_parameter_value().double_value

        self.declare_parameter('weight_spr_goal',0.05)
        self.weight_spr_goal = self.get_parameter('weight_spr_goal').get_parameter_value().double_value

        self.declare_parameter('score_obstacle',2.0)
        self.score_obstacle = self.get_parameter('score_obstacle').get_parameter_value().double_value

        self.declare_parameter('max_acc',3.0)
        self.max_acc = self.get_parameter('max_acc').get_parameter_value().double_value

        self.declare_parameter('max_ang_acc',10.0)
        self.max_ang_acc = self.get_parameter('max_ang_acc').get_parameter_value().double_value

        self.declare_parameter('lim_max_vel',0.7)
        self.lim_max_vel = self.get_parameter('lim_max_vel').get_parameter_value().double_value

        self.declare_parameter('lim_min_vel',0.1)
        self.lim_min_vel = self.get_parameter('lim_min_vel').get_parameter_value().double_value

        self.declare_parameter('lim_max_ang_vel',0.78539)
        self.lim_max_ang_vel = self.get_parameter('lim_max_ang_vel').get_parameter_value().double_value

        self.declare_parameter('topic_global_frame','odom')
        self.topic_global_frame = self.get_parameter('topic_global_frame').get_parameter_value().string_value

        self.declare_parameter('topic_imu','odom')
        self.topic_imu = self.get_parameter('topic_imu').get_parameter_value().string_value

        self.declare_parameter('base_link_frame','base_link')
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value

        if self.topic_global_frame == 'odom':
            print('self.topic_global_frame = ', self.topic_global_frame)
            self.sub = self.create_subscription(Odometry, self.topic_global_frame, self.listener_callback_odom, self.qos)
        elif self.topic_global_frame == '/mavros/local_position/pose':
            print('self.topic_global_frame = ', self.topic_global_frame)                    
            self.sub = self.create_subscription(PoseStamped, self.topic_global_frame, self.listener_callback_PoseStamped, self.qos) 
        elif self.topic_global_frame == 'gps_indoor':
            print('self.topic_global_frame = ', self.topic_global_frame)                    
            self.sub = self.create_subscription(PoseStamped, self.topic_global_frame, self.listener_callback_PoseStamped, self.qos) 
            self.sub = self.create_subscription(Imu, self.topic_imu, self.listener_callback_Imu, self.qos) 

        self.sub = self.create_subscription(Path, "predect_line", self.listener_callback_spraying_line, self.qos)
        self.sub = self.create_subscription(Marker, "obstacles", self.listener_callback_obstacle, self.qos)
        self.subscription = self.create_subscription(OccupancyGrid, 'occupancy_grid_topic', self.occupancyGrid_callback, qos_profile_sensor_data)

        self.cfg = Config(
                    T = self.T,
                    dt = self.samplingtime,
                    num_vel_rollouts = self.num_vel_rollouts,
                    num_ang_vel_rollouts = self.num_ang_vel_rollouts,
                    seed = 1)
        
        self.dwa_params = dict(robot_threshold = self.robot_threshold,
                               goal_tolerance = self.goal_tolerance,
                               obstacle_tolerance = self.dis_safe_obstacle,
                               map_resolution = self.map_resolution,
                               map_origin_x = self.map_origin_x,
                               map_origin_y = self.map_origin_y,
                               
                               weight_angle_row = self.weight_angle_row,
                               weight_angle_turn = self.weight_angle_turn,
                               weight_angle_goal = self.weight_angle_goal,
                               weight_vel = self.weight_vel,
                               weight_obs = self.weight_obs,
                               weight_spr_row = self.weight_spr_row,
                               weight_spr_turn = self.weight_spr_turn,
                               weight_spr_goal = self.weight_spr_goal,
                               score_obstacle = self.score_obstacle,

                               max_acc = self.max_acc,
                               max_ang_acc = self.max_ang_acc,
                               lim_max_vel = self.lim_max_vel,
                               lim_min_vel = self.lim_min_vel,
                               lim_max_ang_vel = self.lim_max_ang_vel)
        
        # print(self.dwa_params)
        self.dwa_planner = DWA_Numba(self.cfg)
        self.dwa_planner.setup(self.dwa_params)

        self.timer = self.create_timer(0.01, self.timer_callback)  # Timer for periodic execution
        
        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 3)
        self.local_path_publisher = self.create_publisher(Path, '/local_path', 10)
        self.sampled_path_publisher = self.create_publisher(MarkerArray, '/sampled_path', 10)
        self.traj_path_publisher = self.create_publisher(Path, '/traj_path', 10)
        self.mileage_publisher = self.create_publisher(Float64, '/mileage', 3)


    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to euler angles
        t0 = +2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        t1 = +1.0 - 2.0 * (quaternion.x**2 + quaternion.y**2)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        t4 = +1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def listener_callback_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        roll, pitch, yaw  = self.quaternion_to_euler(msg.pose.pose.orientation)

        self.read_pos_data = True
        self.read_ang_data = True
        self.current_state=np.array([x, y , yaw])

    def listener_callback_PoseStamped(self,data):
        if self.topic_global_frame == 'gps_indoor':
            self.current_state[0] = data.pose.position.x
            self.current_state[1] = data.pose.position.y
            self.read_pos_data = True

        if self.topic_global_frame == '/mavros/local_position/pose':
            self.current_state[0] = data.pose.position.x
            self.current_state[1] = data.pose.position.y
            self.read_pos_data = True
            roll, pitch, yaw  = self.quaternion_to_euler(data.pose.orientation)
            self.current_state[2] = yaw + np.pi/2
            self.read_ang_data = True

    def listener_callback_Imu(self,data):
        roll, pitch, yaw  = self.quaternion_to_euler(data.orientation)
        self.current_state[2] = yaw
        self.read_ang_data = True

    def listener_callback_spraying_line(self, msg):
        x = []
        y = []
        for pose in msg.poses:
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)
        self.spraying_line = np.array(list(zip(x,y)))

    def listener_callback_obstacle(self, msg):
        self.obstacle_positions = np.array([[point.x, point.y] for point in msg.points])
        self.obstacle_radius = [0.01] * len(self.obstacle_positions)
        self.obstacle_radius = np.array(self.obstacle_radius)

    def occupancyGrid_callback(self, msg):
        self.map_size_x = int(2.0* abs(self.map_origin_x) / self.map_resolution)
        self.map_size_y = int(2.0* abs(self.map_origin_y) / self.map_resolution)
        assert self.map_size_x == int(np.sqrt(len(msg.data))), "Size wrong !"
        assert self.map_size_y == int(np.sqrt(len(msg.data))), "Size wrong !"
        self.costmap = np.array(msg.data).reshape(int(np.sqrt(len(msg.data))), int(np.sqrt(len(msg.data))))
        self.obstacle_radius = np.array( [0.01] * len(self.obstacle_positions))

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

    def _traj_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'  # Set the appropriate frame ID # odom base_link
        x_value = [item[0] for item in self.traj_paths]
        y_value = [item[1] for item in self.traj_paths]
        for i in range(len(x_value)):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = float(x_value[i])
            pose_stamped.pose.position.y = float(y_value[i])
            # Set other pose information as needed
            path_msg.poses.append(pose_stamped)

        # Publish the Path message
        self.traj_path_publisher.publish(path_msg)

    def timer_callback(self):
        """
        Timer callback function that gets executed periodically.
        """
        time1 = time.time()
        twist = Twist()
        self.current_useq = self.pre_vel
        if self.read_pos_data == True and self.read_ang_data == True:
            distance = np.linalg.norm(np.array([self.current_state[0], self.current_state[1]]) - np.array([self.goal[self.goal_num][0], self.goal[self.goal_num][1]]))
            self.robot_position=True
            if distance < self.goal_shift:
                if self.goal_num < len(self.goal)-1:
                    self.goal_num += 1
                    self.gx= self.goal[self.goal_num][0]
                    self.gy= self.goal[self.goal_num][1]
                    self.robot_position=True
                else:    
                    self.robot_position=False
        
        if self.robot_position==True:           
            self.cur_goal =  np.array(self.goal[self.goal_num])
            
            self.dwa_planner.shift_and_update(self.current_state, self.current_useq, self.costmap,
                                            self.obstacle_positions, self.obstacle_radius, self.spraying_line,
                                            self.cur_goal, self.goal_num, num_shifts=1)
            useq = self.dwa_planner.solve()

            twist.linear.x = float(useq[0])
            twist.angular.z = float(useq[1])

            # Get rollout states from subset of maps for visualization? (e.g., 50)
            self.rollout_states_vis, self.opt_states_vis = self.dwa_planner.get_state_rollout()
            cost, cost_head, cost_vel, cost_obs, cost_spray, warning_obstable = self.dwa_planner.get_cost_info()

            opt_states_vis_x = [point[0] for point in self.opt_states_vis]
            opt_states_vis_y = [point[1] for point in self.opt_states_vis]
            self.publish_path(opt_states_vis_x,opt_states_vis_y,self.local_path_publisher,"base_link")
            # self.sampled_path(self.rollout_states_vis[:,:,0].T,self.rollout_states_vis[:,:,1].T, warning_obstable,"base_link")

            # # self.traj_paths.append([self.current_state[0], self.current_state[1]])
            # # self._traj_path()
            if not self.cal_mileage:
                self.pre_state = np.array([self.current_state[0], self.current_state[1] , self.current_state[2]])
                self.cal_mileage = True
            dis = np.linalg.norm(self.current_state[:2] - self.pre_state[:2])
            self.mileage += dis
            self.pre_state = self.current_state
            msg = Float64()
            msg.data = self.mileage
            self.mileage_publisher.publish(msg)
            # print(self.mileage)


            self.pre_vel = np.array(useq.tolist())
            self.read_pos_data =False
            self.read_ang_data = False

            if self.print_data:print(f'robot: x = {self.current_state[0]:.3f}, y = {self.current_state[1]:.3f}, th = {self.current_state[2]:.3f}')
            if self.print_data:print(f'vel: v = {self.pre_vel[0]:.3f}, w = {self.pre_vel[1]:.3f}')
            if self.print_data:print("goal = ", self.cur_goal )

        self.pub_vel.publish(twist)
        if self.print_data:print('time = ', time.time() - time1)
        if self.print_data:print("===============================================")




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