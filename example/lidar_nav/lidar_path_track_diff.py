import irsim
import sys
import numpy as np
from RDA_planner.mpc import MPC
from collections import namedtuple
import cv2
from sklearn.cluster import DBSCAN

# environment
env = irsim.make(save_ani=False, display=True, full=False)
car = namedtuple('car', 'G h cone_type wheelbase max_speed max_acce dynamics')
obs = namedtuple('obstacle', 'center radius vertex cone_type velocity')

# saved ref path
npy_path = sys.path[0] + '/path_track_ref.npy'
ref_path_list = list(np.load(npy_path, allow_pickle=True))
env.draw_trajectory(ref_path_list, traj_type='-k') # plot path


def scan_box(state, scan_data):

    ranges = np.array(scan_data['ranges'])
    angles = np.linspace(scan_data['angle_min'], scan_data['angle_max'], len(ranges))

    point_list = []
    obstacle_list = []

    for i in range(len(ranges)):
        scan_range = ranges[i]
        angle = angles[i]

        if scan_range < ( scan_data['range_max'] - 0.01):
            point = np.array([ [scan_range * np.cos(angle)], [scan_range * np.sin(angle)]  ])
            point_list.append(point)

    if len(point_list) < 4:
        return obstacle_list

    else:
        point_array = np.hstack(point_list).T
        labels = DBSCAN(eps=2.0, min_samples=6).fit_predict(point_array)

        for label in np.unique(labels):
            if label == -1:
                continue
            else:
                point_array2 = point_array[labels == label]
                rect = cv2.minAreaRect(point_array2.astype(np.float32))
                box = cv2.boxPoints(rect)

                vertices = box.T

                trans = state[0:2]
                rot = state[2, 0]
                R = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])
                global_vertices = trans + R @ vertices

                obstacle_list.append(obs(None, None, global_vertices, 'Rpositive', 0))

        return obstacle_list


        # obs_list : obstacle: (center, radius, vertex, cone_type, velocity)

def main():
    
    robot_info = env.get_robot_info()
    car_tuple = car(robot_info.G, robot_info.h, robot_info.cone_type, robot_info.wheelbase, [10, 1], [10, 0.5], 'diff')
    
    mpc_opt = MPC(car_tuple, ref_path_list, receding=10, sample_time=env.step_time, process_num=4, iter_num=2, max_edge_num=4, max_obs_num=3, obstacle_order=True, wu=1.0, slack_gain=10)
    
    for i in range(500):   
        
        scan_data = env.get_lidar_scan()
        # obs_list : obstacle: (center, radius, vertex, cone_type, velocity)
        obs_list = scan_box(env.robot.state, scan_data)

        for obs in obs_list:
            env.draw_box(obs.vertex, refresh=True)
   
        opt_vel, info = mpc_opt.control(env.robot.state, 4, obs_list)
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