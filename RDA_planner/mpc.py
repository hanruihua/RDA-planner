
'''
RDA MPC 
Author: Han Ruihua
'''

import numpy as np
from math import inf, sqrt, pi, sin, cos, tan
from RDA_planner.rda_solver import RDA_solver
import yaml

from collections import namedtuple

rdaobs = namedtuple('rdaobs', 'A b cone_type center vertex')

class MPC:
    def __init__(self, car_tuple, ref_path, receding=10, sample_time=0.1, iter_num=4, enable_reverse=False, rda_obstacle=False, obstacle_order=False, max_edge_num=5, max_obs_num=5, process_num=4, original=False, **kwargs) -> None:

        '''
        Agruments 
            () -- default value; * -- recommended to tune to improve the performance.

            car_tuple: 'G h cone_type wheelbase max_speed max_acce'
            ref_path: a list of reference points, each point is a 3*1 vector (x, y, theta). if enable_reverse is True, the reference path should be splitted by the gear change.
            *receding (10): The receding horizon for mpc.
            sample_time (0.1): the step time of the world.
            *iter_num (4): The maximum number of iterations in mpc.
            enable_reverse (False): If true, the car-robot can move forward and backward, 
                            and the reference path would be splitted in the change of direction.

            rda_obstacle: if True, the obstacle list can be transported to rda_solver directly, otherwise, it should be converted.
            obstacle_order: if True, the obstacle list is ordered by the minimum distance to the robot, otherwise, it is not ordered.
            max_edge_num (5): The maximum number of edges for the polygon obstacle considered in the rda_solver. 
            max_obs_num (5): The maximum number of obstacles considered in the rda_solver.
            process_num (4): The number of processes to solve the rda problem. Depends on your computer
            original: if True, Original ADMM is used in the rda_solver, otherwise, the accelerated ADMM is used.

            kwargs:
                *slack_gain (8): slack gain value for l1 regularization, see paper for details.
                *max_sd (1.0): maximum safety distance.
                *min_sd (0.1): minimum safety distance.
                *ro1 (200): The penalty parameter in ADMM.
                ro2 (1): The penalty parameter in ADMM.
                iter_threshold (0.2): The threshold to stop the iteration. 
                ws (1): The wight for the state difference cost.
                wu (1): The weight for the speed difference cost.
                init_vel ([0,0]): The initial velocity of the car robot.
        '''
        
        self.car_tuple = car_tuple # car_tuple: 'G h cone_type wheelbase max_speed max_acce'
        self.L = car_tuple.wheelbase  # wheel base

        self.receding = receding
        self.dt = sample_time

        self.cur_vel_array = kwargs.get('init_vel', np.zeros((2, receding)))

        self.state = np.zeros((3, 1))

        # flag
        self.cur_index = 0
        self.ref_path = ref_path
        
        self.rda = RDA_solver(receding, car_tuple, max_edge_num, max_obs_num, iter_num=iter_num, step_time=sample_time, process_num=process_num, original=original, **kwargs)

        self.enable_reverse = enable_reverse

        self.rda_obstacle = rda_obstacle
        self.obstacle_order= obstacle_order

        if enable_reverse:
            self.curve_list = self.split_path(self.ref_path)
            self.curve_index = 0


    # @classmethod
    # def init_from_yaml(cls, config_file, **kwargs):

    #     # car_tuple, ref_path, receding=10, sample_time=0.1, iter_num=4, enable_reverse=False, rda_obstacle=False, obstacle_order=False, **kwargs

    #     abs_path = file_check(config_file)
    #     with open(abs_path, 'r') as f:
    #         config = yaml.safe_load(f)
    #         config.update(kwargs)
        
    #     return cls.init_from_robot(**config)


    # @classmethod
    # def init_from_robot():
    #     pass

    def control(self, state, ref_speed=5, obstacle_list=[], **kwargs):

        '''
        state: the robot state (x, y, theta) of current time, 3*1 vector 
        ref_speed: the reference speed, scalar value
        obstacle_list: a list of obstacle
            obstacle: (center, radius, vertex, cone_type, velocity)
        '''

        if np.shape(state)[0] > 3:
            state = state[0:3]

        self.state = state

        if self.enable_reverse:
            cur_ref_path = self.curve_list[self.curve_index]
            gear_flag = cur_ref_path[0][-1, 0]
        else:
            cur_ref_path = self.ref_path
            gear_flag = 1

        state_pre_array, ref_traj_list, self.cur_index = self.pre_process(state, cur_ref_path, self.cur_index, ref_speed, **kwargs)

        if not self.rda_obstacle:
            rda_obs_list = self.convert_rda_obstacle(obstacle_list, state, self.obstacle_order)
        else:
            rda_obs_list = obstacle_list
        
        u_opt_array, info = self.rda.iterative_solve(state_pre_array, self.cur_vel_array, ref_traj_list, gear_flag*ref_speed, rda_obs_list, **kwargs)

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

    def convert_rda_obstacle(self, obstacle_list, state=None, obstacle_order=False):
        rda_obs_list = []

        for obs in obstacle_list:
            if obs.cone_type == 'norm2':
                A, b = self.convert_inequal_circle(obs.center, obs.radius, obs.velocity)

                rda_obs = rdaobs(A, b, obs.cone_type, obs.center, None)
                rda_obs_list.append(rda_obs)

            elif obs.cone_type == 'Rpositive':
                A, b = self.convert_inequal_polygon(obs.vertex, obs.velocity)

                rda_obs = rdaobs(A, b, obs.cone_type, None, obs.vertex)
                rda_obs_list.append(rda_obs)
            
        

        if obstacle_order:
            rda_obs_list.sort(key=self.rda_obs_distance)


        return rda_obs_list

    def rda_obs_distance(self, rda_obs):

        if rda_obs.cone_type == 'norm2':
            distance= MPC.distance(self.state[0:2], rda_obs.center[0:2])
        
        elif rda_obs.cone_type == 'Rpositive':
            distance = np.min(np.linalg.norm(self.state[0:2]-rda_obs.vertex, axis=0))

        return distance
            

    def update_ref_path(self, ref_path):
        self.ref_path = ref_path
        self.cur_index = 0

        if self.enable_reverse:
            self.curve_list = self.split_path(self.ref_path)
            self.curve_index = 0

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


    def convert_inequal_circle(self, center, radius, velocity=np.zeros((2, 1))):
        # center: 2*1
        # radius: scalar
        
        if np.linalg.norm(velocity) <= 0.01:
            A = np.array([ [1, 0], [0, 1], [0, 0] ])
            b = np.row_stack((center, -radius* np.ones((1,1))))
        else:
            A = []
            b = []
            for t in range(self.receding+1):
                next = center + velocity * (t * self.dt)
                temp_A = np.array([ [1, 0], [0, 1], [0, 0] ])
                temp_b = np.row_stack((next, -radius* np.ones((1,1))))

                A.append(temp_A)
                b.append(temp_b)

        return A, b
        

    def convert_inequal_polygon(self, vertex, velocity=np.zeros((2, 1))):
        # vertex: 2*4 matrix

        if np.linalg.norm(velocity) <= 0.01:
            A, b = self.gen_inequal_global(vertex)
        else:
            A, b = [], []

            for t in range(self.receding+1): 
                next = vertex + velocity * (t * self.dt)
                temp_A, temp_b = self.gen_inequal_global(next)
                A.append(temp_A)
                b.append(temp_b)

        return A, b

    def gen_inequal_global(self, vertex):

        temp_vertex = np.c_[vertex, vertex[0:2, 0]]   

        point_num = vertex.shape[1]
        
        A = np.zeros((point_num, 2))
        b = np.zeros((point_num, 1))

        for i in range(point_num):
            cur_p = temp_vertex[0:2, i]
            next_p = temp_vertex[0:2, i+1]

            diff = next_p - cur_p

            ax = diff[1]
            by = -diff[0]
            c = ax * cur_p[0] + by * cur_p[1]

            A[i, 0] = ax
            A[i, 1] = by
            b[i, 0] = c

        return A, b 



