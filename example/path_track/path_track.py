from ir_sim.env import EnvBase
import sys
import numpy as np
from RDA_planner.mpc import MPC
from collections import namedtuple

# environment
env = EnvBase('path_track.yaml', save_ani=False, display=True, full=False)
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce')

# saved ref path
npy_path = sys.path[0] + '/path_track_ref.npy'
ref_path_list = list(np.load(npy_path, allow_pickle=True))
env.draw_trajectory(ref_path_list, traj_type='-k') # plot path

def main():
    obs_list = env.get_obstacle_list()
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.shape[2], [10, 1], [10, 0.5])
    mpc_opt = MPC(car_tuple, obs_list, ref_path_list, sample_time=env.step_time)
    
    for i in range(500):   
        opt_vel, info = mpc_opt.control(env.robot.state, ref_speed=4)
        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)

        env.step(opt_vel, stop=False)
        env.render(show_traj=True, show_trail=True)

        if env.done():
            env.render_once(show_traj=True, show_trail=True)
            break

        if info['arrive']:
            print('arrive at the goal')
            break

    env.end(ani_name='path_track', show_traj=True, show_trail=True, ending_time=10, ani_kwargs={'subrectangles':True})
    
if __name__ == '__main__':
    main()