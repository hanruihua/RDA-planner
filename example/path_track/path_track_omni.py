import irsim
import sys
import numpy as np
from RDA_planner.mpc import MPC
from collections import namedtuple
import time
from math import cos, sin

# environment
env = irsim.make(save_ani=False, display=True, full=False)
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce dynamics')

# saved ref path
npy_path = sys.path[0] + '/path_track_ref.npy'
ref_path_list = list(np.load(npy_path, allow_pickle=True))
env.draw_trajectory(ref_path_list, traj_type='-k') # plot path

def main():
    
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.shape[2], [10, 6.28], [5, 2], 'omni')
    
    mpc_opt = MPC(car_tuple, ref_path_list, receding=10, sample_time=env.step_time, process_num=4, iter_num=3, obstacle_order=True, ro1=300, max_edge_num=4, max_obs_num=11, slack_gain=15, max_sd=1.0, ws=1.0, wu=1.0)
    
    for i in range(500):   
        
        obs_list = env.get_obstacle_info_list()
        opt_vel, info = mpc_opt.control(env.robot.state[0:3], 4, obs_list)

        # mpc_opt.rda.assign_adjust_parameter(ro1=100, ro2=1)

        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)
        env.draw_trajectory(info['ref_traj_list'], 'y', refresh=True)

        vx = opt_vel[0, 0] * cos(opt_vel[1, 0])
        vy = opt_vel[0, 0] * sin(opt_vel[1, 0])
        omni_vel = np.array([[vx], [vy]])

        env.step(omni_vel)
        # env.step(opt_vel, stop=False)
    
        env.render(show_traj=True, show_trail=True)

        if env.done():
            break

        if info['arrive']:
            print('arrive at the goal')
            break

    env.end(ani_name='path_track_omni', show_traj=True, show_trail=True, ending_time=10, ani_kwargs={'subrectangles':True})
    
if __name__ == '__main__':
    main()