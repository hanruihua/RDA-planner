import irsim
import sys
import numpy as np
from RDA_planner.mpc import MPC
from collections import namedtuple
import time

# environment

env = irsim.make(save_ani=False, display=True, full=False)
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce dynamics')

# saved ref path
npy_path = sys.path[0] + '/path_track_ref.npy'
ref_path_list = list(np.load(npy_path, allow_pickle=True))
env.draw_trajectory(ref_path_list, traj_type='-k') # plot path

def main():
    
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.shape[2], [10, 1], [10, 0.5], 'diff')
    
    mpc_opt = MPC(car_tuple, ref_path_list, receding=10, sample_time=env.step_time, process_num=4, iter_num=2, obstacle_order=True, ro1=300, max_edge_num=4, max_obs_num=11, slack_gain=8)
    
    for i in range(500):   
        
        obs_list = env.get_obstacle_info_list()
        opt_vel, info = mpc_opt.control(env.robot.state[0:3], 4, obs_list)

        # mpc_opt.rda.assign_adjust_parameter(ro1=100, ro2=1)

        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)

        env.step(opt_vel)
        env.render(show_traj=True, show_trail=True)

        if env.done():
            break

        if info['arrive']:
            print('arrive at the goal')
            break

    env.end(ani_name='path_track_diff', show_traj=True, show_trail=True, ending_time=10)
    
if __name__ == '__main__':
    main()