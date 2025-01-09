import irsim
import sys
import numpy as np
from RDA_planner.mpc import MPC
from collections import namedtuple
import time
from gctl.curve_generator import curve_generator

# environment
env = irsim.make(save_ani=False, display=True, full=False)
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce dynamics')

# saved ref path
start_point = np.array([[0], [25], [0]])
goal_point = np.array([[59], [25], [0]])
point_list = [start_point, goal_point]
cg = curve_generator()
ref_path_list = cg.generate_curve('dubins', point_list, 0.1, 5)

# npy_path = sys.path[0] + '/path_track_ref.npy'
# ref_path_list = list(np.load(npy_path, allow_pickle=True))
env.draw_trajectory(ref_path_list, traj_type='-k') # plot path


def main():
    
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.wheelbase, [10, 1], [10, 1.0], 'acker')
    
    mpc_opt = MPC(car_tuple, ref_path_list, receding=10, sample_time=env.step_time, process_num=4, iter_num=2, obstacle_order=True, ro1=300, max_edge_num=4, max_obs_num=11, slack_gain=8, lobca=True) 
                         
    for i in range(500):   
        
        obs_list = env.get_obstacle_list()
        # print('obs_list:', obs_list)
        opt_vel, info = mpc_opt.control(env.robot.state, 4, obs_list, ws=0.1, wst=0.1, max_sd=2.0)

        # mpc_opt.rda.assign_adjust_parameter(ro1=100, ro2=1)

        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)

        env.step(opt_vel)
        env.render(show_traj=True, show_trail=True)

        if env.done():
            break

        if info['arrive']:
            print('arrive at the goal')
            break

    env.end(ani_name='path_track', show_traj=True, show_trail=True, ending_time=10, ani_kwargs={'subrectangles':True})
    
if __name__ == '__main__':
    main()