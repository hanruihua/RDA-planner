"""
RDA MPC 
Author: Han Ruihua (hanrh@connect.hku.hk)
"""

import numpy as np
from math import inf, sqrt, pi, sin, cos, tan
from RDA_planner.rda_solver import RDA_solver
import yaml
from collections import namedtuple

rdaobs = namedtuple("rdaobs", "A b cone_type center vertex")


class MPC:
    """
    Arguments
        () -- default value; * -- recommended to tune to improve the performance.

        car_tuple (tuple): 'G h cone_type wheelbase max_speed max_acce dynamics',  dynamics: acker, diff, or omni
            dynamics (str): 'acker', 'diff', or 'omni'. Default is 'acker'.
            - 'acker': velocity is a 2D vector (linear speed, steering angle).
            - 'diff': velocity is a 2D vector (linear speed, angular speed).
            - 'omni': velocity is a 2D vector (linear speed, velocity angle).

        ref_path (list of 3*1 vectors): A list of reference points, each point is a 3*1 vector (x, y, theta).
            If enable_reverse is True, the reference path would be splitted by the gear change, and the each point should be (4*1) vectors. Default is an empty list [].

        *receding (int, default=10): The receding horizon for mpc.
        *iter_num (int, default=4): The maximum number of iterations in mpc.

        sample_time (float, default=0.1): The step time of the world.

        enable_reverse (bool, default=False): If true, the car-robot can move forward and backward,
                        and the reference path would be splitted in the change of direction.

        rda_obstacle (bool, default=False): If True, the obstacle list can be transported to rda_solver directly, otherwise, it should be converted.

        obstacle_order (bool, default=True): If True, the obstacle list is ordered by the minimum distance to the robot, otherwise, it is not ordered.

        max_edge_num (int, default=5): The maximum number of edges for the polygon obstacle considered in the rda_solver.

        max_obs_num (int, default=5): The maximum number of obstacles considered in the rda_solver.

        process_num (int, default=4): The number of processes to solve the rda problem in parallel. Please set this value based on the CPU core.

        accelerated (bool, default=True): Controls the use of accelerated ADMM in the rda_solver.
            If True, acceleration is applied. If False, consider reducing ro1 (e.g., ro1=1) and
            increasing iter_num (e.g., add 5) to improve stability and ensure convergence.

        time_print (bool, default=False): If True, the time cost of each iteration will be printed.

        goal_index_threshold (int, default=1): The threshold to determine if the robot arrives at the goal by the index of the reference path.

        kwargs:
            *slack_gain (float, default=8): Slack gain value for l1 regularization, see paper for details.
            *max_sd (float, default=1.0): Maximum safety distance.
            *min_sd (float, default=0.1): Minimum safety distance.
            *ro1 (int, default=200): The penalty parameter in ADMM.
            ro2 (int, default=1): The penalty parameter in ADMM.
            iter_threshold (float, default=0.2): The threshold to stop the iteration.
            ws (float, default=1.0): The weight for the state difference cost.
            wu (float, default=1.0): The weight for the speed difference cost.
            init_vel (list of 2 floats, default=[0.0,0.0]): The initial velocity of the robot.
    """

    def __init__(
        self,
        car_tuple,
        ref_path,
        receding: int = 10,
        sample_time: float = 0.1,
        iter_num: int = 4,
        enable_reverse: bool = False,
        rda_obstacle: bool = False,
        obstacle_order: bool = True,
        max_edge_num: int = 5,
        max_obs_num: int = 5,
        process_num: int = 4,
        accelerated: bool = True,
        time_print: bool = False,
        goal_index_threshold: int = 1,
        **kwargs,
    ) -> None:

        self.car_tuple = (
            car_tuple  # car_tuple: 'G h cone_type wheelbase max_speed max_acce'
        )
        self.L = car_tuple.wheelbase  # wheel base
        self.dynamics = car_tuple.dynamics

        self.receding = receding
        self.dt = sample_time

        self.cur_vel_array = kwargs.get("init_vel", np.zeros((2, receding)))

        self.state = np.zeros((3, 1))

        # flag
        self.cur_index = 0
        self.ref_path = ref_path

        self.rda = RDA_solver(
            receding,
            car_tuple,
            max_edge_num,
            max_obs_num,
            iter_num=iter_num,
            step_time=sample_time,
            process_num=process_num,
            accelerated=accelerated,
            time_print=time_print,
            **kwargs,
        )

        self.enable_reverse = enable_reverse

        self.rda_obstacle = rda_obstacle
        self.obstacle_order = obstacle_order

        self.goal_index_threshold = goal_index_threshold

        if enable_reverse:
            self.curve_list = self.split_path(self.ref_path)
            self.curve_index = 0

    def control(self, state, ref_speed=5, obstacle_list=[], **kwargs):
        """
        state: the robot state (x, y, theta) of current time, 3*1 vector
        ref_speed: the reference speed, scalar value
        obstacle_list: a list of obstacle
            obstacle: (center, radius, vertex, cone_type, velocity)
        """
 
        if np.shape(state)[0] > 3: state = state[0:3]
            
        self.state = state

        if self.enable_reverse:
            cur_ref_path = self.curve_list[self.curve_index]
            gear_flag = cur_ref_path[0][-1, 0]
        else:
            cur_ref_path = self.ref_path
            gear_flag = 1

        state_pre_array, ref_traj_list, self.cur_index = self.pre_process(
            state, cur_ref_path, self.cur_index, ref_speed, **kwargs
        )

        if not self.rda_obstacle:
            rda_obs_list = self.convert_rda_obstacle(
                obstacle_list, state, self.obstacle_order
            )
        else:
            rda_obs_list = obstacle_list

        u_opt_array, info = self.rda.iterative_solve(
            state_pre_array,
            self.cur_vel_array,
            ref_traj_list,
            gear_flag * ref_speed,
            rda_obs_list,
            **kwargs,
        )

        if self.cur_index >= len(cur_ref_path) - self.goal_index_threshold:

            if self.enable_reverse:
                self.curve_index += 1
                self.cur_index = 0

                if self.curve_index < len(self.curve_list):
                    cur_ref_path = self.curve_list[self.curve_index]
                    info["arrive"] = False
                else:
                    u_opt_array = np.zeros((2, self.receding))
                    info["arrive"] = True
            else:
                u_opt_array = np.zeros((2, self.receding))
                info["arrive"] = True

        else:
            info["arrive"] = False

        self.cur_vel_array = u_opt_array

        return u_opt_array[:, 0:1], info

    def convert_rda_obstacle(self, obstacle_list, state=None, obstacle_order=False):
        rda_obs_list = []

        for obs in obstacle_list:
            if obs.cone_type == "norm2":
                A, b = self.convert_inequal_circle(obs.center, obs.radius, obs.velocity)

                rda_obs = rdaobs(A, b, obs.cone_type, obs.center, None)
                rda_obs_list.append(rda_obs)

            elif obs.cone_type == "Rpositive":
                A, b = self.convert_inequal_polygon(obs.vertex, obs.velocity)

                rda_obs = rdaobs(A, b, obs.cone_type, None, obs.vertex)
                rda_obs_list.append(rda_obs)

        if obstacle_order:
            rda_obs_list.sort(key=self.rda_obs_distance)

        return rda_obs_list

    def rda_obs_distance(self, rda_obs):

        if rda_obs.cone_type == "norm2":
            distance = MPC.distance(self.state[0:2], rda_obs.center[0:2])

        elif rda_obs.cone_type == "Rpositive":
            distance = np.min(np.linalg.norm(self.state[0:2] - rda_obs.vertex, axis=0))

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

        for index, point in enumerate(ref_path):
            if point[-1, 0] != flag:

                curve_list.append(ref_path[start_index:index])
                start_index = index
                flag = point[-1, 0]

        curve_list.append(ref_path[start_index:])

        return curve_list

    def pre_process(self, state, ref_path, cur_index, ref_speed, **kwargs):
        # find closest points
        min_dis, min_index = self.closest_point(state, ref_path, cur_index, **kwargs)

        # predict the state list and find the reference points list
        cur_state = state
        traj_point = ref_path[min_index]

        ref_traj_list = [traj_point]
        state_pre_list = [cur_state]

        for i in range(self.receding):

            if self.dynamics == "acker":
                cur_state = self.motion_predict_model_acker(
                    cur_state, self.cur_vel_array[:, i : i + 1], self.L, self.dt
                )
            elif self.dynamics == "diff":
                cur_state = self.motion_predict_model_diff(
                    cur_state, self.cur_vel_array[:, i : i + 1], self.dt
                )

            elif self.dynamics == "omni":
                cur_state = self.motion_predict_model_omni(
                    cur_state, self.cur_vel_array[:, i : i + 1], self.dt
                )

            state_pre_list.append(cur_state)

            move_len = ref_speed * self.dt
            traj_point, cur_index = self.inter_point(
                traj_point, ref_path, cur_index, move_len
            )

            diff = traj_point[2, 0] - cur_state[2, 0]
            traj_point[2, 0] = cur_state[2, 0] + MPC.wraptopi(diff)
            ref_traj_list.append(traj_point)

        state_pre_array = np.hstack(state_pre_list)

        return state_pre_array, ref_traj_list, min_index

    def motion_predict_model_acker(self, car_state, vel, wheel_base, sample_time):

        assert car_state.shape == (3, 1) and vel.shape == (2, 1)

        phi = car_state[2, 0]

        v = vel[0, 0]
        psi = vel[1, 0]
        ds = np.array([[v * cos(phi)], [v * sin(phi)], [v * tan(psi) / wheel_base]])

        next_state = car_state + ds * sample_time

        return next_state

    def motion_predict_model_diff(self, robot_state, vel, sample_time):

        assert robot_state.shape == (3, 1) and vel.shape == (2, 1)

        phi = robot_state[2, 0]
        v = vel[0, 0]
        w = vel[1, 0]

        ds = np.array([[v * cos(phi)], [v * sin(phi)], [w]])

        next_state = robot_state + ds * sample_time

        # next_state[2, 0] = wraptopi(next_state[2, 0])

        return next_state

    def motion_predict_model_omni(self, robot_state, vel, sample_time):

        assert robot_state.shape[0] >= 2 and vel.shape == (2, 1)

        vx = vel[0, 0] * cos(vel[1, 0])
        vy = vel[0, 0] * sin(vel[1, 0])
        omni_vel = np.array([[vx], [vy], [0]])
        # omni_vel = np.array([[vx], [vy]])

        next_state = robot_state + sample_time * omni_vel
        # ds = np.row_stack((vel, [0]))
        # next_state = robot_state + sample_time * ds

        return next_state

    def closest_point(
        self, state, ref_path, start_ind, threshold=0.1, ind_range=10, **kwargs
    ):

        min_dis = inf
        min_ind = start_ind

        for i, waypoint in enumerate(ref_path[start_ind : start_ind + ind_range]):
            dis = MPC.distance(state[0:2], waypoint[0:2])
            if dis < min_dis:
                min_dis = dis
                min_ind = start_ind + i
                if dis < threshold:
                    break

        return min_dis, min_ind

    def inter_point(self, traj_point, ref_path, cur_ind, length):

        circle = np.squeeze(traj_point[0:2])
        new_traj_point = np.copy(traj_point)

        while True:

            if cur_ind + 1 > len(ref_path) - 1:

                end_point = ref_path[-1]
                end_point[2] = MPC.wraptopi(end_point[2])

                return end_point, cur_ind

            cur_point = ref_path[cur_ind]
            next_point = ref_path[cur_ind + 1]

            segment = [np.squeeze(cur_point[0:2]), np.squeeze(next_point[0:2])]
            int_point = self.range_cir_seg(circle, length, segment)

            if int_point is None:
                cur_ind = cur_ind + 1
            else:
                diff = MPC.wraptopi(next_point[2, 0] - cur_point[2, 0])
                theta = MPC.wraptopi(cur_point[2, 0] + diff / 2)
                new_traj_point[0:2, 0] = int_point[:]
                new_traj_point[2, 0] = theta

                return new_traj_point, cur_ind

    def range_cir_seg(self, circle, r, segment):

        assert (
            circle.shape == (2,)
            and segment[0].shape == (2,)
            and segment[1].shape == (2,)
        )

        sp = segment[0]
        ep = segment[1]

        d = ep - sp

        if np.linalg.norm(d) == 0:
            return None

        # if d.all() == 0:
        #     return None

        f = sp - circle

        a = d @ d
        b = 2 * f @ d
        c = f @ f - r**2

        discriminant = b**2 - 4 * a * c

        if discriminant < 0:
            return None
        else:

            t1 = (-b - sqrt(discriminant)) / (2 * a)
            t2 = (-b + sqrt(discriminant)) / (2 * a)

            if t2 >= 0 and t2 <= 1:
                int_point = sp + t2 * d
                return int_point

            return None

    @staticmethod
    def distance(point1, point2):
        return sqrt(
            (point1[0, 0] - point2[0, 0]) ** 2 + (point1[1, 0] - point2[1, 0]) ** 2
        )

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
            A = np.array([[1, 0], [0, 1], [0, 0]])
            b = np.row_stack((center, -radius * np.ones((1, 1))))
        else:
            A = []
            b = []
            for t in range(self.receding + 1):
                next_center = center + velocity * (t * self.dt)
                temp_A = np.array([[1, 0], [0, 1], [0, 0]])
                temp_b = np.row_stack((next_center, -radius * np.ones((1, 1))))

                A.append(temp_A)
                b.append(temp_b)

        return A, b

    def convert_inequal_polygon(self, vertex, velocity=np.zeros((2, 1))):
        # vertex: 2*4 matrix

        if np.linalg.norm(velocity) <= 0.01:
            A, b = self.gen_inequal_global(vertex)
        else:
            A, b = [], []

            for t in range(self.receding + 1):
                next_vertex = vertex + velocity * (t * self.dt)
                temp_A, temp_b = self.gen_inequal_global(next_vertex)
                A.append(temp_A)
                b.append(temp_b)

        return A, b

    def gen_inequal_global(self, vertex):

        convex_flag, order = self.is_convex_and_ordered(vertex)

        # assert convex_flag, f'The polygon constructed by vertex is not convex. Please check the vertex: {vertex}'
        if not convex_flag:
            print(
                f"Warning: The polygon constructed by vertex is not convex. Please check the vertex: {vertex}"
            )

        if order == "CW":
            vertex = vertex[:, ::-1]

        temp_vertex = np.c_[vertex, vertex[0:2, 0]]

        point_num = vertex.shape[1]

        A = np.zeros((point_num, 2))
        b = np.zeros((point_num, 1))

        for i in range(point_num):
            cur_p = temp_vertex[0:2, i]
            next_p = temp_vertex[0:2, i + 1]

            diff = next_p - cur_p

            ax = diff[1]
            by = -diff[0]
            c = ax * cur_p[0] + by * cur_p[1]

            A[i, 0] = ax
            A[i, 1] = by
            b[i, 0] = c

        return A, b

    def cross_product(self, o, a, b):
        """Compute the cross product of vectors OA and OB.
        A positive cross product indicates a counter-clockwise turn,
        a negative indicates a clockwise turn, and zero indicates a collinear point."""
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def is_convex_and_ordered(self, points):
        """Determine if the polygon is convex and return the order (CW or CCW).

        Args:
            points (np.ndarray): A 2xN NumPy array representing the vertices of the polygon.

        Returns:
            (bool, str): A tuple where the first element is True if the polygon is convex,
                        and the second element is 'CW' or 'CCW' based on the order.
                        If not convex, returns (False, None).
        """
        n = points.shape[1]  # Number of points
        if n < 3:
            return False, None  # A polygon must have at least 3 points

        # Initialize the direction for the first cross product
        direction = 0

        for i in range(n):
            o = points[:, i]
            a = points[:, (i + 1) % n]
            b = points[:, (i + 2) % n]

            cross = self.cross_product(o, a, b)

            if cross != 0:  # Only consider non-collinear points
                if direction == 0:
                    direction = 1 if cross > 0 else -1
                elif (cross > 0 and direction < 0) or (cross < 0 and direction > 0):
                    return False, None  # Not convex

        return True, "CCW" if direction > 0 else "CW"

    def get_adjust_parameters(self):
        """
        get the adjust parameter of the rda_solver: ws, wu, ro1, ro2, iter_threshold, slack_gain, max_sd, min_sd

        """

        return self.rda.get_adjust_parameter()


    def no_ref_path(self):
        return len(self.ref_path) == 0


    def reset(self):
        self.cur_vel_array = np.zeros((2, self.receding))
        self.cur_index = 0
        self.curve_index = 0

        self.rda.reset()
