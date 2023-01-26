from ir_sim.env import EnvBase
import sys
import numpy as np
from RDA_planner.mpc import MPC
from collections import namedtuple
import matplotlib.pyplot as plt
import time
from GCT.curve_generator import curve_generator

start_point = np.array([[0], [20], [0]])
goal_point = np.array([[60], [20], [0]])

point_list = [start_point, goal_point]

cg = curve_generator(select_mode='default', x_limit = [0, 50], y_limit=[0, 50])
ref_path_list = cg.generate_curve('dubins', point_list, 0.1, 5, include_gear=True)
# cg.plot_curve(ref_path_list, show_direction=False)

# plt.close()

init_point = ref_path_list[0][0:3]
add_dim = np.array([[0]])
init_point = np.vstack((init_point, add_dim))
env = EnvBase('corridor.yaml', save_ani=False, full=False, display=True, robot_args={'state': init_point, 'goal':goal_point})
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce')

env.draw_trajectory(ref_path_list, traj_type='-k')
# env.show()

if __name__ == '__main__':

    obs_list = env.get_obstacle_list()
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.shape[2], [10, 1], [10, 0.01])
    mpc_opt = MPC(car_tuple, obs_list, ref_path_list, receding=10, sample_time=env.step_time, iter_num=4, enable_reverse=False, iter_threshold=0.2, process_num=4, slack_gain=8, max_sd=1.0, min_sd=0.1, ws=1, wu=1, ro1=200, ro2=1)
    
    for i in range(500):   
        
        opt_vel, info = mpc_opt.control(env.robot.state, ref_speed=4)

        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)

        env.step(opt_vel)
        env.render(0.0001, show_traj=True, show_trail=True)
        # env.render(0.1, show_traj=True)
        if env.done():
            break
        
        if info['arrive']:
            print('arrive at the goal')
            break

    env.end(ani_name='corridor', show_traj=True, show_trail=True, rm_fig_path=True, ending_time=1000, ani_kwargs={'subrectangles':True})
    
