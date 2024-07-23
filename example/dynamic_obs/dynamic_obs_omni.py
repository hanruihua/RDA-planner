import sys
import time
from collections import namedtuple
import numpy as np
<<<<<<< HEAD
from irsim.env import EnvBase
=======
from ir_sim.env import EnvBase
>>>>>>> add omni dynamics
from RDA_planner.mpc import MPC
from math import cos, sin

# environment
env = EnvBase('dynamic_obs_omni.yaml', save_ani=False, display=True, full=False)
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce dynamics')

# saved ref path
npy_path = sys.path[0] + '/dynamic_obs.npy'
ref_path_list = list(np.load(npy_path, allow_pickle=True))
env.draw_trajectory(ref_path_list, traj_type='-k') # plot path

def main():
    
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.wheelbase, [10, 6.28], [10, 1.5], 'omni')
    
    mpc_opt = MPC(car_tuple, ref_path_list, receding=10, sample_time=env.step_time, process_num=5, iter_num=2, max_edge_num=4, max_obs_num=6, min_sd=0.2, wu=0.5, ws=0.5, obstacle_order=True)
    
    for i in range(500):   
        
        obs_list = env.get_obstacle_list()
        opt_vel, info = mpc_opt.control(env.robot.state, 6, obs_list)
        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)

        vx = opt_vel[0, 0] * cos(opt_vel[1, 0])
        vy = opt_vel[0, 0] * sin(opt_vel[1, 0])
        omni_vel = np.array([[vx], [vy]])

        env.step(omni_vel, stop=False)
        env.render(show_traj=True)

        if env.done():
            break

        if info['arrive']:
            print('arrive at the goal')
            break

    env.end(ani_name='dynamic_obs', show_traj=True, show_trail=True, ending_time=3, ani_kwargs={'subrectangles':True})
    
if __name__ == '__main__':
    main()