from ir_sim.env import EnvBase
import sys
import numpy as np
from RDA_planner.mpc import MPC
from collections import namedtuple
import matplotlib.pyplot as plt
import time
from GCT.curve_generator import curve_generator

#custom
point1 = np.array([ [5], [40], [0]])
point2 = np.array([ [35], [11], [-3.14]])
point3 = np.array([ [43.8], [11], [-3.14]])

point_list = [point1, point2, point3]

cg = curve_generator(select_mode='default', x_limit = [0, 50], y_limit=[0, 50])
ref_path_list = cg.generate_curve('reeds', point_list, 0.5, 5, include_gear=True)

init_point = ref_path_list[0][0:3]
add_dim = np.array([[0]])
init_point = np.vstack((init_point, add_dim))
env = EnvBase('reverse.yaml', save_ani=False, display=True, robot_args={'state': init_point, 'goal':point3})
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce')

env.draw_trajectory(ref_path_list, traj_type='-k')
# env.show()

if __name__ == '__main__':

    obs_list = env.get_obstacle_list()
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.shape[2], [10, 1], [10, 0.5])
    mpc_opt = MPC(car_tuple, obs_list, ref_path_list, sample_time=env.step_time, enable_reverse=True)
    
    for i in range(500):   
        
        if np.linalg.norm(env.robot.state[0:2] - point2[0:2]) <= 1:
            print('arrive point2')
            mpc_opt.update_parameter(max_sd=0.1, min_sd=0.1, slack_gain=1)

        opt_vel, info = mpc_opt.control(env.robot.state, ref_speed=4)

        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)

        env.step(opt_vel, stop=False)
        env.render(0.0001, show_traj=True, show_trail=True)
        # env.render(0.1, show_traj=True)
        if env.done():
            break
        
        if info['arrive']:
            print('arrive at the goal')
            break

    env.end(ani_name='reverse_park', show_traj=True, show_trail=True, rm_fig_path=True, ending_time=10, ani_kwargs={'subrectangles':True})
    
