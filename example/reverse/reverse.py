from ir_sim.env import EnvBase
import numpy as np
from RDA_planner.mpc import MPC
from collections import namedtuple
from GCT.curve_generator import curve_generator

env = EnvBase('reverse.yaml', save_ani=False, display=True, full=False)

# start and goal point of the robot
point1 = np.array([ [5], [40], [0]])
point2 = np.array([ [35], [11], [-3.14]])
point3 = np.array([ [43.8], [11], [-3.14]])
point_list = [env.robot.state[0:3], point1, point2, point3]

cg = curve_generator()
ref_path_list = cg.generate_curve('reeds', point_list, 0.5, 5, include_gear=True)

car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce')

env.draw_trajectory(ref_path_list, traj_type='-k')

if __name__ == '__main__':

    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.shape[2], [10, 1], [10, 0.5])

    obstacle_template_list = [{'edge_num': 3, 'obstacle_num': 4, 'cone_type': 'norm2'}, {'edge_num': 4, 'obstacle_num': 3, 'cone_type': 'Rpositive'}]  # define the number of obstacles in advance

    mpc_opt = MPC(car_tuple, ref_path_list, sample_time=env.step_time, enable_reverse=True, obstacle_template_list=obstacle_template_list)
    
    for i in range(500):   
        
        if np.linalg.norm(env.robot.state[0:2] - point2[0:2]) <= 1:
            print('arrive point2')
            mpc_opt.update_parameter(max_sd=0.1, min_sd=0.1, slack_gain=1)

        obs_list = env.get_obstacle_list()
        opt_vel, info = mpc_opt.control(env.robot.state, 4, obs_list)

        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)

        env.step(opt_vel, stop=False)
        env.render(0.0001, show_traj=True, show_trail=True)
        # env.render(0.1, show_traj=True)
         
        if info['arrive']:
            print('arrive at the goal')
            break
        
        if env.done():
            print('done')
            break

    env.end(ani_name='reverse_park', show_traj=True, show_trail=True, ending_time=10, ani_kwargs={'subrectangles':True})
    
