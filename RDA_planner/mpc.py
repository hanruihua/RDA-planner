
'''
RDA MPC 
Author: Han Ruihua
'''

import numpy as np
from math import inf, sqrt, pi, sin, cos, tan
from RDA_planner.rda_solver import RDA_solver
import time

class MPC:
    def __init__(self, car_tuple, ref_path, receding=10, sample_time=0.1, iter_num=4, enable_reverse=False, **kwargs) -> None:

        '''
        Agruments 
            () -- default value; * -- recommended to tune to improve the perfromance.

            *receding (10): The receding horizon for mpc.
            sample_time (0.1): the step time of the world.
            *iter_num (4): The maximum number of iterations in mpc.
            enable_reverse (False): If true, the car-robot can move forward and backward, 
                            and the reference path would be splitted in the change of direction.
            iter_threshold (0.2): The threshold to stop the iteration. 
            process_num (4): The number of processes to solve the rda problem. Depends on your computer
            *slack_gain (8): slack gain value for l1 regularization, see paper for details.
            *max_sd (1.0): maximum safety distance.
            *min_sd (0.1): minimum safety distance.
            ws (1): The wight for the state difference cost.
            wu (1): The weight for the speed difference cost.
            *ro1 (200): The penalty parameter in ADMM.
            ro2 (1): The penalty parameter in ADMM.
            init_vel ([0,0]): The initial velocity of the car robot.
        '''
        
        self.car_tuple = car_tuple # car_tuple: 'G h cone wheelbase max_speed max_acce'
        self.L = car_tuple.wheelbase  # wheel base

        self.receding = receding
        self.dt = sample_time

        self.cur_vel_array = kwargs.get('init_vel', np.zeros((2, receding)))

        # flag
        self.cur_index = 0
        self.ref_path = ref_path

        start_time = time.time()
        self.rda = RDA_solver(receding, car_tuple, iter_num=iter_num, **kwargs)
        print( time.time() - start_time)

        self.enable_reverse = enable_reverse

        if enable_reverse:
            self.curve_list = self.split_path(self.ref_path)
            self.curve_index = 0

    def control(self, state,  ref_speed=5, obstacle_list=[],**kwargs):

        '''
        state: the robot state (x, y, theta) of current time, 3*1 vector 
        ref_speed: the reference speed, scalar value
        obstacle_list: a list of obstacle
            obstacle: (vertex point; cone_type, )
        '''




        if np.shape(state)[0] > 3:
            state = state[0:3]

        if self.enable_reverse:
            cur_ref_path = self.curve_list[self.curve_index]
            gear_flag = cur_ref_path[0][-1, 0]
        else:
            cur_ref_path = self.ref_path
            gear_flag = 1

        state_pre_array, ref_traj_list, self.cur_index = self.pre_process(state, cur_ref_path, self.cur_index, ref_speed, **kwargs)




        u_opt_array, info = self.rda.iterative_solve(state_pre_array, self.cur_vel_array, ref_traj_list, gear_flag*ref_speed, obs_list, **kwargs)

        if self.cur_index == len(cur_ref_path) - 1:

            if self.enable_reverse:
                self.curve_index += 1
                self.cur_index = 0

                if self.curve_index < len(self.curve_list):
                    cur_ref_path = self.curve_list[self.curve_index]
                    info['arrive'] = False
                else:
                    u_opt_array = np.zeros((2, self.receding))
                    info['arrive'] = True
            else:
                u_opt_array = np.zeros((2, self.receding))
                info['arrive'] = True

        else:
            info['arrive'] = False
        
        self.cur_vel_array = u_opt_array

        return u_opt_array[:, 0:1], info

    def update_ref_path(self, ref_path):
        self.ref_path = ref_path
        self.cur_index = 0

    def update_parameter(self, **kwargs):
        self.rda.assign_adjust_parameter(**kwargs)

    def split_path(self, ref_path):
        # split path by gear

        flag = ref_path[0][-1, 0]
        curve_list = []

        start_index = 0
        end_index = -1

        for index, point in enumerate(ref_path):
            if point[-1, 0] != flag:

                end_index = index
                curve_list.append(ref_path[start_index:end_index])
                start_index = end_index
                flag = point[-1, 0]

        curve_list.append(ref_path[end_index:])

        return curve_list

    def pre_process(self, state, ref_path, cur_index, ref_speed, **kwargs):
        # find closest points 
        min_dis, min_index = self.closest_point(state, ref_path, cur_index, **kwargs)

        # predict the state list and find the reference points list
        cur_state = state
        traj_point = ref_path[min_index]

        ref_traj_list = [ traj_point ]
        state_pre_list = [cur_state]

        for i in range(self.receding):
            cur_state = self.motion_predict_model(cur_state, self.cur_vel_array[:, i:i+1], self.L, self.dt)
            state_pre_list.append(cur_state)

            move_len = ref_speed * self.dt
            traj_point, cur_index = self.inter_point(traj_point, ref_path, cur_index, move_len)

            diff = traj_point[2, 0] - cur_state[2, 0]
            traj_point[2, 0] = cur_state[2, 0] + MPC.wraptopi(diff)
            ref_traj_list.append( traj_point )

        state_pre_array = np.hstack(state_pre_list)

        return state_pre_array, ref_traj_list, min_index

    def motion_predict_model(self, car_state, vel, wheel_base, sample_time):

        assert car_state.shape == (3, 1) and vel.shape == (2, 1) 

        phi = car_state[2, 0]

        v = vel[0, 0]
        psi = vel[1, 0]

        ds = np.array([ [v*cos(phi)], [v*sin(phi)], [ v*tan(psi) / wheel_base ] ])  
    
        next_state = car_state + ds * sample_time

        return next_state

    def closest_point(self, state, ref_path, start_ind, threshold=0.1, ind_range=10, **kwargs):
        
        min_dis = inf
        min_ind = start_ind
    
        for i, waypoint in enumerate(ref_path[start_ind:start_ind+ind_range]):
            dis = MPC.distance(state[0:2], waypoint[0:2])
            if dis < min_dis:
                min_dis = dis
                min_ind = start_ind + i
                if dis < threshold: break
                    
        return min_dis, min_ind

    def inter_point(self, traj_point, ref_path, cur_ind, length):

        circle = np.squeeze(traj_point[0:2])
        new_traj_point = np.copy(traj_point)

        while True:

            if cur_ind+1 > len(ref_path) - 1:
                
                end_point = ref_path[-1]
                end_point[2] = MPC.wraptopi(end_point[2])

                return end_point, cur_ind
            
            cur_point = ref_path[cur_ind]
            next_point = ref_path[cur_ind + 1]

            segment = [np.squeeze(cur_point[0:2]) , np.squeeze(next_point[0:2])]
            int_point = self.range_cir_seg(circle, length, segment)

            if int_point is None:
                cur_ind = cur_ind + 1
            else:
                diff = MPC.wraptopi(next_point[2, 0] - cur_point[2, 0])
                theta = MPC.wraptopi(cur_point[2, 0] + diff / 2 )
                new_traj_point[0:2, 0] = int_point[:]
                new_traj_point[2, 0] = theta
                
                return new_traj_point, cur_ind

    def range_cir_seg(self, circle, r, segment):
        
        assert circle.shape == (2,) and segment[0].shape == (2,) and segment[1].shape == (2,)

        sp = segment[0]
        ep = segment[1]

        d = ep - sp

        if np.linalg.norm(d) == 0:
            return None

        # if d.all() == 0:
        #     return None

        f = sp - circle

        a = d @ d
        b = 2* f@d
        c = f@f - r ** 2

        discriminant = b**2 - 4 * a * c

        if discriminant < 0:
            return None
        else:
            
            t1 = (-b - sqrt(discriminant)) / (2*a)
            t2 = (-b + sqrt(discriminant)) / (2*a)
  
            if t2 >= 0 and t2 <=1:
                int_point = sp + t2 * d
                return int_point

            return None

    @staticmethod
    def distance(point1, point2):
        return sqrt( (point1[0, 0] - point2[0, 0])**2 + (point1[1, 0] - point2[1, 0])**2 )

    @staticmethod
    def wraptopi(radian):
        while radian > pi:
            radian = radian - 2 * pi
        while radian < -pi:
            radian = radian + 2 * pi

        return radian