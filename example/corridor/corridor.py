from ir_sim.env import EnvBase
import numpy as np
from RDA_planner.mpc import MPC
from collections import namedtuple
from GCT.curve_generator import curve_generator

# start and goal point of the robot
start_point = np.array([[0], [20], [0]])
goal_point = np.array([[60], [20], [0]])
point_list = [start_point, goal_point]

# generate dubins curve
cg = curve_generator()
ref_path_list = cg.generate_curve('dubins', point_list, 0.1, 5)

# init simulated environment
robot_init_point = np.zeros((4, 1))
robot_init_point[0:3] = ref_path_list[0][0:3]

env = EnvBase('corridor.yaml', save_ani=False, full=False, display=True, robot_args={'state': robot_init_point, 'goal':goal_point})
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce')  # robot information

env.draw_trajectory(ref_path_list, traj_type='-k')

if __name__ == '__main__':

    obs_list = env.get_obstacle_list()
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.shape[2], [10, 1], [10, 0.5])
    mpc_opt = MPC(car_tuple, obs_list, ref_path_list, slack_gain=8, sample_time=env.step_time)
    
    for i in range(500):   
        
        opt_vel, info = mpc_opt.control(env.robot.state, ref_speed=4)

        env.draw_trajectory(info['opt_state_list'], 'r', refresh=True)

        env.step(opt_vel)
        env.render(show_traj=True, show_trail=True)
        
        if env.done():
            break
        
        if info['arrive']:
            print('arrive at the goal')
            break

    env.end(ani_name='corridor', show_traj=True, show_trail=True, ending_time=10, ani_kwargs={'subrectangles':True})
    
