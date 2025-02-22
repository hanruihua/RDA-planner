import sys
import time
from collections import namedtuple
import numpy as np
import irsim
from RDA_planner.mpc import MPC

# environment
env = irsim.make(save_ani=False, display=True, full=False)
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce dynamics')

# saved ref path
npy_path = sys.path[0] + '/dynamic_obs.npy'
ref_path_list = list(np.load(npy_path, allow_pickle=True))
env.draw_trajectory(ref_path_list, traj_type='-k') # plot path

def main():
    
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.wheelbase, [10, 1], [10, 1.0], 'acker')
    
    mpc_opt = MPC(car_tuple, ref_path_list, receding=10, sample_time=env.step_time, process_num=5, iter_num=2, max_edge_num=4, max_obs_num=6, min_sd=0.5, wu=0.2, obstacle_order=True)
    
    for i in range(500):   
        
        obs_list = env.get_obstacle_info_list()
        opt_vel, info = mpc_opt.control(env.robot.state, 6, obs_list)
        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)

        env.step(opt_vel)
        env.render(show_traj=True)

        if env.done():
            break

        if info['arrive']:
            print('arrive at the goal')
            break

    env.end(ani_name='dynamic_obs', show_traj=True, show_trail=True, ending_time=3, ani_kwargs={'subrectangles':True})
    
if __name__ == '__main__':
    main()