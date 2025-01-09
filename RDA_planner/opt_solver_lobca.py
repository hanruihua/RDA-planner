import cvxpy as cp
import numpy as np
from multiprocessing import Pool
from math import sin, cos, tan, inf
from pathlib import Path
import time

pool = None

class opt_solver:

    def __init__(self, receding, car_tuple, obstacle_list, iter_num=2, step_time=0.1, iter_threshold=0.1, **kwargs) -> None:
        
        # setting
        self.T = receding
        self.car_tuple = car_tuple 
        self.L = car_tuple.wheelbase
        self.obstacle_list = obstacle_list
        self.iter_num = iter_num
        self.dt = step_time
        self.acce_bound = np.c_[car_tuple.max_acce] * self.dt 
        self.max_speed = np.c_[self.car_tuple.max_speed]
        self.iter_threshold = iter_threshold

        # independ variable
        self.indep_s = cp.Variable((3, receding+1), name='state')
        self.indep_u = cp.Variable((2, receding), name='vel')
        self.gamma_dis = cp.Variable((receding, ), nonneg=True)

        self.update_obstecles(obstacle_list)

        # flag
        self.init_flag = True
            
    def update_obstecles(self, obstacle_list):

        self.obstacle_list = obstacle_list

        self.indep_v_list = [cp.Variable((1, self.T+1)) for obs in obstacle_list ]
        self.indep_w_list = [cp.Variable((self.T+1, 2)) for obs in obstacle_list ]
        self.indep_lam_list = [ cp.Variable((obs.A.shape[0], self.T+1)) for obs in obstacle_list ]
        self.indep_mu_list = [ cp.Variable((self.car_tuple.G.shape[0], self.T+1)) for obs in obstacle_list ]

        self.nom_lam_list = [ np.ones((obs.A.shape[0], self.T+1)) for obs in obstacle_list ]
        self.nom_mu_list = [ np.ones((self.car_tuple.G.shape[0], self.T+1)) for obs in obstacle_list ]
        self.nom_y_list = [ np.zeros((self.T+1, 2)) for obs in obstacle_list] 

    def iterative_solve(self, state_pre_array, cur_vel_array, ref_traj_list, ref_speed, algorithm, **kwargs):
        
        for i in range(self.iter_num):

            start_time = time.time()

            opt_state_array, opt_velocity_array = self.lobca_prob(ref_traj_list, ref_speed, state_pre_array, cur_vel_array, **kwargs)

            if np.linalg.norm( opt_velocity_array - cur_vel_array) <= self.iter_threshold:
                print('iteration early stop')
                break 
            
            state_pre_array = opt_state_array
            cur_vel_array = opt_velocity_array
            
        print(algorithm + ' iteration time: ', time.time() - start_time)

        # info for debug
        opt_state_list = [state[:, np.newaxis] for state in state_pre_array.T ]
        info = {'ref_traj_list': ref_traj_list, 'opt_state_list': opt_state_list}

        return opt_velocity_array, info 

    def nav_prob(self, ref_state, ref_speed, nom_s, nom_u, **kwargs):
        cost, constraints = self.nav_cost_cons(ref_state, ref_speed, nom_s, nom_u, **kwargs)
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.ECOS, verbose=False)

        if prob.status == cp.OPTIMAL:
            return self.indep_s.value, self.indep_u.value

        else:
            print('solve failed, do not update state and velocity')
            return nom_s, nom_u
    
    def lobca_prob(self, ref_state, ref_speed, nom_s, nom_u, **kwargs):
        nav_cost, nav_constraints = self.nav_cost_cons(ref_state, ref_speed, nom_s, nom_u, **kwargs)
        lobca_cost, lobca_constraints = self.lobca_cost_cons(nom_s, **kwargs)

        cost = nav_cost + lobca_cost
        constraints = nav_constraints + lobca_constraints
        
        
        prob = cp.Problem(cp.Minimize(cost), constraints)

        start_time = time.time() 
        prob.solve(solver=cp.ECOS, verbose=False)
        print('problem run time: ', time.time() - start_time, 'problem solve time: ', prob.solver_stats.solve_time)
        if prob.status == cp.OPTIMAL:

            for j in range(len(self.nom_lam_list)):
                self.nom_lam_list[j] = self.indep_lam_list[j].value

            return self.indep_s.value, self.indep_u.value
        else:
            print('solve failed, do not update state and velocity')
            return nom_s, nom_u


    def nav_cost_cons(self, ref_state, ref_speed, nom_s, nom_u, ws=1, wst=1, wu=1, wut=1, **kwargs):


        cost = 0
        constraints = []

        temp_s_list = []

        cost += wu * cp.sum_squares(self.indep_u[0, 0:self.T-1] - ref_speed)
        cost += wut * cp.square(self.indep_u[0, self.T-1] - ref_speed) # cost speed at T
        # cost += self.indep_s - ref_st1
        ref_s = np.hstack(ref_state)
        
        cost += ws * cp.sum_squares( self.indep_s[:, 0:self.T-1] - ref_s[:, 0:self.T-1])
        cost += wst * cp.sum_squares( self.indep_s[:, self.T-1] - ref_s[:, self.T-1])

        for t in range(self.T):
            indep_st = self.indep_s[:, t:t+1]
            indep_st1 = self.indep_s[:, t+1:t+2]
            indep_ut = self.indep_u[:, t:t+1]

            nom_st = nom_s[:, t:t+1]
            nom_ut = nom_u[:, t:t+1]

            ref_st1 = ref_state[t+1]
            ref_ut = ref_speed

            A, B, C = self.linear_ackermann_model(nom_st, nom_ut, self.dt, self.L)  # taylor expansion 
            temp_s_list.append(A @ indep_st + B @ indep_ut + C)

        temp_s_array = cp.hstack(temp_s_list)
        constraints += [ self.indep_s[:, 1:] == temp_s_array ]
        # A_array = 
        # temp = indep_u[:, 1:] - indep_u[:, :-1]
        
        constraints += [ cp.abs(self.indep_u[:, 1:] - self.indep_u[:, :-1] ) <= self.acce_bound ]  # constraints on speed accelerate
        constraints += [ cp.abs(self.indep_u) <= self.max_speed]
        constraints += [ self.indep_s[:, 0:1] == nom_s[:, 0:1] ] # constraint on init state  
        
        # constraints += [ self.indep_u[0, :] >= 0.5] # constraints on max speed

        return cost, constraints

    def lobca_cost_cons(self, nom_s, slack_gain=15, ro=50, c1=0.1, c2=0.1, c3=0.1, max_sd=1.0, min_sd=0.1, **kwargs):
        
        cost = 0
        constraints = []

        cost += (-slack_gain * cp.sum(self.gamma_dis) )

        ## obstacle constraints interior
        for obs_index, obs in enumerate(self.obstacle_list): 
            indep_v_array = self.indep_v_list[obs_index]
            indep_w_array = self.indep_w_list[obs_index]
            indep_lam_array = self.indep_lam_list[obs_index]
            nom_lam_array = self.nom_lam_list[obs_index]
            indep_mu_array = self.indep_mu_list[obs_index]

            Talor_lam_A_t_list = []
            Talor_lam_A_R_list = []

            for t in range(self.T):             
                indep_lam = indep_lam_array[:, t+1:t+2]
                nom_lam = nom_lam_array[:, t+1:t+2]

                indep_trans = self.indep_s[:, t+1:t+2][0:2]
                nom_trans = nom_s[:, t+1:t+2][0:2]
                nom_phi = nom_s[:, t+1:t+2][2, 0]
                indep_phi = self.indep_s[:, t+1:t+2][2, 0]

                nom_rot = np.array( [[cos(nom_phi), -sin(nom_phi)],  [sin(nom_phi), cos(nom_phi)]] )
                dnom_rot = np.array( [[-sin(nom_phi), -cos(nom_phi)],  [cos(nom_phi), -sin(nom_phi)]] )

                Talor_lam_A_t = indep_lam.T @ obs.A @ nom_trans + nom_lam.T @ obs.A @ indep_trans - nom_lam.T @ obs.A @ nom_trans
                indep_rot = nom_rot + dnom_rot * (indep_phi - nom_phi)
                Talor_lam_A_R = indep_lam.T @ obs.A @ nom_rot + nom_lam.T @ obs.A @ indep_rot - nom_lam.T @ obs.A @ nom_rot

                Talor_lam_A_t_list.append(Talor_lam_A_t)
                Talor_lam_A_R_list.append(Talor_lam_A_R)

            constraints += [ cp.diag(indep_v_array - (indep_lam_array.T @ obs.b) - (indep_mu_array.T @ self.car_tuple.h))[1:] >= self.gamma_dis]
            constraints += [ cp.constraints.zero.Zero(indep_mu_array.T @ self.car_tuple.G + indep_w_array)  ]
            constraints += [ cp.norm( obs.A.T @ indep_lam_array, axis=0) <= 1 ]
            constraints += [ self.cone_cp_array(-indep_lam_array, obs.cone_type) ]
            constraints += [ self.cone_cp_array(-indep_mu_array, self.car_tuple.cone_type) ]
            
            Talor_lam_A_t_array = cp.hstack(Talor_lam_A_t_list)
            Talor_lam_A_R_array = cp.vstack(Talor_lam_A_R_list)

            temp1 = ro * cp.sum_squares( indep_v_array[:, 1:] - Talor_lam_A_t_array)
            temp2 = ro * cp.sum_squares(indep_w_array[1:, :] - Talor_lam_A_R_array)

            temp3 = 0.5 * c1 * cp.sum_squares(indep_lam_array - nom_lam_array)
            temp4 = 0.5 * c2 * cp.sum_squares(self.indep_s - nom_s)

            cost += (temp1 + temp2 + temp3 + temp4)

        constraints += [ cp.max(self.gamma_dis) <= max_sd ] 
        constraints += [ cp.min(self.gamma_dis) >= min_sd ] 

        return cost, constraints

    def linear_ackermann_model(self, nom_state, nom_u, dt, L):
        
        phi = nom_state[2, 0]
        v = nom_u[0, 0]
        psi = nom_u[1, 0]

        A = np.array([ [1, 0, -v * dt * sin(phi)], [0, 1, v * dt * cos(phi)], [0, 0, 1] ])

        B = np.array([ [cos(phi)*dt, 0], [sin(phi)*dt, 0], 
                        [ tan(psi)*dt / L, v*dt/(L * (cos(psi))**2 ) ] ])

        C = np.array([ [ phi*v*sin(phi)*dt ], [ -phi*v*cos(phi)*dt ], 
                        [ -psi * v*dt / ( L * (cos(psi))**2) ]])

        return A, B, C

    def cone_cp_array(self, array, cone='Rpositive'):
        # cone for cvxpy: R_positive, norm2
        if cone == 'Rpositive':
            return cp.constraints.nonpos.NonPos(array)
        elif cone == 'norm2':
            return cp.constraints.nonpos.NonPos( cp.norm(array[0:-1], axis = 0) - array[-1]  )
