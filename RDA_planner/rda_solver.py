
'''
RDA solver 
Author: Han Ruihua (hanrh@connect.hku.hk)
'''

import cvxpy as cp
import numpy as np
from pathos.multiprocessing import Pool
from math import sin, cos, tan, inf
import time
from collections import namedtuple

# para_obstacle = namedtuple('obstacle', ['At', 'bt', 'cone_type'])
pool = None

class RDA_solver:
    def __init__(self, receding, 
                       car_tuple, 
                       max_edge_num=5, 
                       max_obs_num=5,
                       iter_num=2, step_time=0.1, iter_threshold=0.2, process_num=4, accelerated=True, time_print=True, **kwargs) -> None:

        '''
            kwargs:
                *slack_gain (8): slack gain value for l1 regularization, see paper for details.
                *max_sd (1.0): maximum safety distance.
                *min_sd (0.1): minimum safety distance.
                *ro1 (200): The penalty parameter in ADMM.
                 ro2 (1): The penalty parameter in ADMM.
        '''

        # setting
        self.T = receding
        self.car_tuple = car_tuple # car_tuple: 'G h cone_type wheelbase max_speed max_acce dynamics'
        self.L = car_tuple.wheelbase
        self.max_speed = np.c_[self.car_tuple.max_speed]
        self.max_obs_num = max_obs_num
        self.max_edge_num = max_edge_num
        self.dynamics = car_tuple.dynamics

        self.iter_num = iter_num
        self.dt = step_time
        self.acce_bound = np.c_[car_tuple.max_acce] * self.dt 
        self.iter_threshold = iter_threshold

        self.accelerated = accelerated
        # independ variable and cvxpy parameters definition
        self.definition(**kwargs)

        # flag
        # self.init_flag = True
        self.process_num = process_num

        if process_num == 1:
            self.construct_problem(**kwargs)
        elif process_num > 1:
            global pool 
            pool = self.construct_mp_problem(process_num, **kwargs)
        
        self.time_print = time_print
    
    # region: definition of variables and parameters
    def definition(self, **kwargs):

        self.state_variable_define()
        self.dual_variable_define()
        
        self.state_parameter_define()
        self.dual_parameter_define()

        self.obstacle_parameter_define()

        self.adjust_parameter_define(**kwargs)

        self.combine_parameter_define()

        self.combine_variable_define()


    def state_variable_define(self):
        # decision variables
        self.indep_s = cp.Variable((3, self.T+1), name='state')
        self.indep_u = cp.Variable((2, self.T), name='vel')
        self.indep_dis = cp.Variable((1, self.T), name='distance', nonneg=True)

        self.indep_rot_list = [cp.Variable((2, 2), name='rot_'+str(t))  for t in range(self.T)]  # the variable of rotation matrix

    def dual_variable_define(self):
        
        '''
        define the indep_lam; indep_mu; indep_z
        ''' 
        self.indep_lam_list = []
        self.indep_mu_list = []
        self.indep_z_list = []

        self.indep_lam_list = [cp.Variable((self.max_edge_num, self.T+1), name='lam_' + str(index)) for index in range(self.max_obs_num)]
        self.indep_mu_list = [cp.Variable((self.car_tuple.G.shape[0], self.T+1), name='mu_' + str(index)) for index in range(self.max_obs_num)]
        self.indep_z_list = [cp.Variable((1, self.T), nonneg=True, name='z_' + str(index)) for index in range(self.max_obs_num)]


    def combine_variable_define(self):

        self.indep_Im_array_su = cp.Variable((self.max_obs_num, self.T), name='Im_array_su')
        self.indep_Im_array_LamMuZ =[ cp.Variable((self.T,), name='Im_array_LamMuZ') for i in range(self.max_obs_num)]

        self.indep_Hm_array_su = cp.Variable((self.max_obs_num * self.T, 2), name='Im_array_su')
        self.indep_Hm_array_LamMuZ = [ cp.Variable((self.T, 2), name='Hm_array_LamMuZ') for i in range(self.max_obs_num)]


    def state_parameter_define(self):
        
        self.para_ref_s = cp.Parameter((3, self.T+1), name='para_ref_state')
        self.para_ref_speed = cp.Parameter(name='para_ref_speed')

        self.para_s = cp.Parameter((3, self.T+1), name='para_state')
        self.para_u = cp.Parameter((2, self.T), name='para_vel')
        self.para_dis = cp.Parameter((1, self.T), nonneg=True, value=np.ones((1, self.T)), name='para_dis')

        self.para_rot_list = [cp.Parameter((2, 2), name='para_rot_'+str(t)) for t in range(self.T)]
        self.para_drot_list = [cp.Parameter((2, 2), name='para_drot_'+str(t)) for t in range(self.T)]
        self.para_drot_phi_list = [cp.Parameter((2, 2), name='para_drot_phi_'+str(t)) for t in range(self.T)]

        self.para_A_list = [ cp.Parameter((3, 3), name='para_A_'+str(t)) for t in range(self.T)]
        self.para_B_list = [ cp.Parameter((3, 2), name='para_B_'+str(t)) for t in range(self.T)]
        self.para_C_list = [ cp.Parameter((3, 1), name='para_C_'+str(t)) for t in range(self.T)]

    def dual_parameter_define(self):
        # define the parameters related to obstacles
        self.para_lam_list = []
        self.para_mu_list = []
        self.para_z_list = []
        self.para_xi_list = []
        self.para_zeta_list = []

        for index in range(self.max_obs_num):
            oen = self.max_edge_num
            ren = self.car_tuple.G.shape[0]

            self.para_lam_list += [ cp.Parameter((oen, self.T+1), value=0.1*np.zeros((oen, self.T+1)), name='para_lam_'+ str(oen) + '_'  + str(index)) ]
            self.para_mu_list += [ cp.Parameter((ren, self.T+1), value=np.zeros((ren, self.T+1)), name='para_mu_'+ str(oen) + '_'  + str(index)) ]
            self.para_z_list += [ cp.Parameter((1, self.T), nonneg=True, value=np.zeros((1, self.T)), name='para_z_'+ str(oen) + '_'  + str(index))]
            self.para_xi_list += [ cp.Parameter((self.T+1, 2), value=np.zeros((self.T+1, 2)), name='para_xi_'+ str(oen) + '_'  + str(index))]
            self.para_zeta_list += [ cp.Parameter((1, self.T), value = np.zeros((1, self.T)), name='para_zeta_'+ str(oen) + '_' + str(index))]

        

    def obstacle_parameter_define(self):

        self.para_obstacle_list = []

        for index in range(self.max_obs_num):
            oen = self.max_edge_num

            A_list = [ cp.Parameter((oen, 2), value=np.zeros((oen, 2)), name='para_A_t'+ str(t)) for t in range(self.T+1)]
            b_list = [ cp.Parameter((oen, 1), value=np.zeros((oen, 1)), name='para_b_t'+ str(t)) for t in range(self.T+1)]
            para_cone = cp.Parameter((2, ), value=np.array([0, 1]), nonneg=True, name='para_cone_' + str(index))  # cone type: R_positive [0], norm2 [1]; 
            para_obstacle={'A': A_list, 'b': b_list, 'assign': False, 'cone_type': para_cone}       # cone_type: 'norm2' or 'Rpositive'
            self.para_obstacle_list.append(para_obstacle)


    def combine_parameter_define(self):
        self.para_obsA_lam_list = []   # lam.T @ obsA
        self.para_obsb_lam_list = []   # lam.T @ obsb
        self.para_obsA_rot_list = []   # obs.A @ rot
        self.para_obsA_trans_list = []   # obs.A @ trans

        for index in range(self.max_obs_num):
            oen = self.max_edge_num

            para_obsA_lam = cp.Parameter((self.T+1, 2), value=np.zeros((self.T+1, 2)), name='para_obsA_lam')
            para_obsb_lam = cp.Parameter((self.T+1, 1), value=np.zeros((self.T+1, 1)), name='para_obsb_lam')

            self.para_obsA_lam_list.append(para_obsA_lam)
            self.para_obsb_lam_list.append(para_obsb_lam)

            para_obsA_rot = [ cp.Parameter((oen, 2), value=np.zeros((oen, 2)), name='para_obsA_rot_t'+ str(t)) for t in range(self.T+1) ]
            para_obsA_trans = [ cp.Parameter((oen, 1), value=np.zeros((oen, 1)), name='para_obsA_trans_t'+ str(t)) for t in range(self.T+1) ]

            self.para_obsA_rot_list.append(para_obsA_rot)
            self.para_obsA_trans_list.append(para_obsA_trans)


    def adjust_parameter_define(self, **kwargs):
        # ws: 1
        # wu: 1
        # ro1: 200
        # ro2: 1
        # slack_gain: 8
        # max_sd: 1.0
        # min_sd: 0.1

        # self.para_ws = cp.Parameter(value=1, nonneg=True)
        # self.para_wu = cp.Parameter(value=1, nonneg=True)
        self.para_slack_gain = cp.Parameter(value=kwargs.get('slack_gain', 8), nonneg=True)
        self.para_max_sd = cp.Parameter(value=kwargs.get('max_sd', 1.0), nonneg=True)
        self.para_min_sd = cp.Parameter(value=kwargs.get('min_sd', 0.1), nonneg=True)

        self.ro1 = cp.Parameter(value=kwargs.get('ro1', 200), nonneg=True)
        self.ro2 = cp.Parameter(value=kwargs.get('ro2', 1), nonneg=True)

    # endregion

    # region: construct the problem
    def construct_problem(self, **kwargs):
        self.prob_su = self.construct_su_prob(**kwargs)
        self.prob_LamMuZ_list = self.construct_LamMuZ_prob(**kwargs)


    def construct_mp_problem(self, process_num, **kwargs):
        self.prob_su = self.construct_su_prob(**kwargs)
        pool = Pool(processes=process_num, initializer=self.init_prob_LamMuZ, initargs=(kwargs, )) 
        return pool
    
    def construct_su_prob(self, **kwargs):
        
        self.ws = kwargs.get('ws', 1)
        self.wu = kwargs.get('wu', 1)

        # ro1 = kwargs.get('ro1', 200)
        # ro2 = kwargs.get('ro2', 1)
        
        nav_cost, nav_constraints = self.nav_cost_cons(self.ws, self.wu)
        su_cost, su_constraints = self.update_su_cost_cons(self.para_slack_gain, self.ro1, self.ro2)

        prob_su = cp.Problem(cp.Minimize(nav_cost+su_cost), su_constraints+nav_constraints) 

        assert prob_su.is_dcp(dpp=True)

        return prob_su

    def construct_LamMuZ_prob(self, **kwargs):
        
        # ro1 = kwargs.get('ro1', 200)
        # ro2 = kwargs.get('ro2', 1) 

        prob_list = []

        for obs_index in range(self.max_obs_num):

            indep_lam = self.indep_lam_list[obs_index]
            indep_mu = self.indep_mu_list[obs_index]
            indep_z = self.indep_z_list[obs_index]

            indep_Im_lamMuZ = self.indep_Im_array_LamMuZ[obs_index]
            indep_Hm_lamMuZ = self.indep_Hm_array_LamMuZ[obs_index]

            para_xi = self.para_xi_list[obs_index]
            para_zeta = self.para_zeta_list[obs_index]

            para_obs = self.para_obstacle_list[obs_index]

            para_obsA_rot = self.para_obsA_rot_list[obs_index]
            para_obsA_trans = self.para_obsA_trans_list[obs_index]

            cost, constraints = self.LamMuZ_cost_cons(indep_lam, indep_mu, indep_z, indep_Im_lamMuZ, indep_Hm_lamMuZ, self.para_s, self.para_rot_list, para_xi, self.para_dis, para_zeta, para_obs, para_obsA_rot, para_obsA_trans, self.T, 1, self.ro2)
            
            prob = cp.Problem(cp.Minimize(cost), constraints)

            assert prob.is_dcp(dpp=True)
            
            prob_list.append(prob)

        return prob_list

    def init_prob_LamMuZ(self, kwargs):
        global prob_LamMuZ_list, para_xi_list, para_zeta_list, para_s, para_rot_list, para_dis, para_obsA_rot_list, para_obsA_trans_list, para_obstacle_list, para_cone_norm, para_cone_Rpositive

        para_xi_list = self.para_xi_list
        para_zeta_list = self.para_zeta_list
        para_s = self.para_s
        para_rot_list = self.para_rot_list
        para_dis = self.para_dis 

        para_obsA_rot_list = self.para_obsA_rot_list
        para_obsA_trans_list = self.para_obsA_trans_list
        para_obstacle_list = self.para_obstacle_list

        prob_LamMuZ_list = self.construct_LamMuZ_prob_parallel(para_xi_list, para_zeta_list, para_s, para_rot_list, para_dis, para_obstacle_list, para_obsA_rot_list, para_obsA_trans_list, **kwargs)

    def construct_LamMuZ_prob_parallel(self, para_xi_list, para_zeta_list, para_s, para_rot_list, para_dis, para_obstacle_list, para_obsA_rot_list, para_obsA_trans_list, **kwargs):

        # ro1 = kwargs.get('ro1', 200)
        # ro2 = kwargs.get('ro2', 1) 

        prob_list = []

        for obs_index in range(self.max_obs_num):

            indep_lam = self.indep_lam_list[obs_index]
            indep_mu = self.indep_mu_list[obs_index]
            indep_z = self.indep_z_list[obs_index]

            indep_Im_lamMuZ = self.indep_Im_array_LamMuZ[obs_index] # combine variables
            indep_Hm_lamMuZ = self.indep_Hm_array_LamMuZ[obs_index]

            para_xi = para_xi_list[obs_index]
            para_zeta = para_zeta_list[obs_index]

            para_obs = para_obstacle_list[obs_index]
            para_obsA_rot = para_obsA_rot_list[obs_index]
            para_obsA_trans = para_obsA_trans_list[obs_index]

            cost, constraints = self.LamMuZ_cost_cons(indep_lam, indep_mu, indep_z, indep_Im_lamMuZ, indep_Hm_lamMuZ, para_s, para_rot_list, para_xi, para_dis, para_zeta, para_obs, para_obsA_rot, para_obsA_trans, self.T, 1, self.ro2)
            
            prob = cp.Problem(cp.Minimize(cost), constraints)
            prob_list.append(prob)

        return prob_list


    def nav_cost_cons(self, ws=1, wu=1):
 
        # path tracking objective cost constraints
        # indep_s: cp.Variable((3, self.receding+1), name='state')
        # indep_u: cp.Variable((2, self.receding), name='vel')
        # para_ref_s: cp.Parameter((3, self.T+1), name='para_ref_state')

        cost = 0
        constraints = []

        cost += self.C0_cost(self.para_ref_s, self.para_ref_speed, self.indep_s, self.indep_u, ws, wu)

        constraints += self.dynamics_constraint(self.indep_s, self.indep_u, self.T)
        constraints += self.bound_su_constraints(self.indep_s, self.indep_u, self.para_s, self.max_speed, self.acce_bound)

        return cost, constraints
    
    def update_su_cost_cons(self, slack_gain=8, ro1=200, ro2=1):
        cost = 0
        constraints = []

        if self.max_obs_num == 0:
            return cost, constraints

        cost += self.C1_cost(self.indep_dis, slack_gain)

        Im_su_list = []
        Hm_su_list = []
        
        for obs_index, para_obs in enumerate(self.para_obstacle_list):  
            
            para_xi = self.para_xi_list[obs_index]

            para_lam = self.para_lam_list[obs_index]
            para_mu = self.para_mu_list[obs_index]
            para_z = self.para_z_list[obs_index]
            para_zeta = self.para_zeta_list[obs_index]

            para_obsA_lam = self.para_obsA_lam_list[obs_index]
            para_obsb_lam = self.para_obsb_lam_list[obs_index]

            Imsu = self.Im_su(self.indep_s, self.indep_dis, para_lam, para_mu, para_z, para_zeta, para_obs, para_obsA_lam, para_obsb_lam)
            Hmsu = self.Hm_su(self.indep_rot_list, para_mu, para_lam, para_xi, para_obs, self.T, para_obsA_lam)
            
            Im_su_list.append(Imsu)
            Hm_su_list.append(Hmsu)

        rot_diff_list = []
        for t in range(self.T):

            indep_phi = self.indep_s[2, t+1:t+2]
            indep_rot_t = self.indep_rot_list[t]

            rot_diff_list.append(self.para_rot_list[t] - self.para_drot_phi_list[t] + self.para_drot_list[t] * indep_phi - indep_rot_t)

        rot_diff_array = cp.vstack(rot_diff_list)
        Im_su_array = cp.vstack(Im_su_list)
        Hm_su_array = cp.vstack(Hm_su_list)

        constraints += [cp.constraints.zero.Zero(rot_diff_array)]

        constraints += [self.indep_Im_array_su == Im_su_array]

        if self.accelerated:
            cost += 0.5*ro1 * cp.sum_squares(cp.neg(self.indep_Im_array_su))
        else:
            cost += 0.5*ro1 * cp.sum_squares(self.indep_Im_array_su)
        # constraints += [Im_su_array >= 0]

        constraints += [ self.indep_Hm_array_su == Hm_su_array]
        cost += 0.5*ro2 * cp.sum_squares(self.indep_Hm_array_su)

        constraints += self.bound_dis_constraints(self.indep_dis)

        return cost, constraints

    def LamMuZ_cost_cons(self, indep_lam, indep_mu, indep_z, indep_Im_lamMuZ, indep_Hm_lamMuZ, para_s, para_rot_list, para_xi, para_dis, para_zeta, para_obs, para_obsA_rot, para_obsA_trans, receding, ro1, ro2):

        cost = 0
        constraints = []

        Hm_array = self.Hm_LamMu(indep_lam, indep_mu, para_rot_list, para_xi, para_obs, receding, para_obsA_rot)
        Im_array = self.Im_LamMu(indep_lam, indep_mu, indep_z, para_s, para_dis, para_zeta, para_obs, para_obsA_trans)

        constraints += [indep_Im_lamMuZ == Im_array]

        if self.accelerated:
            cost += 0.5*ro1 * cp.sum_squares(cp.neg(indep_Im_lamMuZ))
        else:
            cost += 0.5*ro1 * cp.sum_squares(indep_Im_lamMuZ)
        # constraints += [ Im_array >= 0 ]

        constraints += [indep_Hm_lamMuZ == Hm_array]
        cost += 0.5*ro2 * cp.sum_squares(indep_Hm_lamMuZ)

        temp_list = []
        for t in range(self.T):
            para_obsAt = para_obs['A'][t+1]
            indep_lam_t = indep_lam[:, t+1:t+2]
            temp_list.append( cp.norm(para_obsAt.T @ indep_lam_t) )
        
        temp = cp.max(cp.vstack(temp_list))

        constraints += [ temp <= 1 ]
        # constraints += [ cp.norm(para_obs.A.T @ indep_lam, axis=0) <= 1 ]
        constraints += [ self.cone_para_array(-indep_lam, para_obs['cone_type']) ]
        constraints += [ self.cone_cp_array(-indep_mu, self.car_tuple.cone_type) ]

        return cost, constraints
    
    # endregion

    # region: assign value for parameters
    def assign_adjust_parameter(self, **kwargs):
        # self.para_ws.value = kwargs.get('ws', 1)
        # self.para_wu.value = kwargs.get('wu', 1) 

        self.para_slack_gain.value = kwargs.get('slack_gain', self.para_slack_gain.value)
        self.para_max_sd.value = kwargs.get('max_sd', self.para_max_sd.value)
        self.para_min_sd.value = kwargs.get('min_sd', self.para_min_sd.value)
        self.ro1.value = kwargs.get('ro1', self.ro1.value)
        self.ro2.value = kwargs.get('ro2', self.ro2.value)

    def assign_state_parameter(self, nom_s, nom_u, nom_dis):

        self.para_s.value = nom_s
        self.para_u.value = nom_u
        self.para_dis.value = nom_dis
        
        for t in range(self.T):
            nom_st = nom_s[:, t:t+1]
            nom_ut = nom_u[:, t:t+1]

            if self.dynamics == 'acker':
                A, B, C = self.linear_ackermann_model(nom_st, nom_ut, self.dt, self.L)
            elif self.dynamics == 'diff':
                A, B, C = self.linear_diff_model(nom_st, nom_ut, self.dt)
            elif self.dynamics == 'omni':
                A, B, C = self.linear_omni_model(nom_ut, self.dt)

            self.para_A_list[t].value = A
            self.para_B_list[t].value = B
            self.para_C_list[t].value = C

            nom_phi = nom_st[2, 0]
            self.para_rot_list[t].value = np.array([[cos(nom_phi), -sin(nom_phi)],  [sin(nom_phi), cos(nom_phi)]])
            self.para_drot_list[t].value = np.array( [[-sin(nom_phi), -cos(nom_phi)],  [cos(nom_phi), -sin(nom_phi)]] )
            self.para_drot_phi_list[t].value = nom_phi * np.array( [[-sin(nom_phi), -cos(nom_phi)],  [cos(nom_phi), -sin(nom_phi)]] )

    def assign_state_parameter_parallel(self, input):

        nom_s, nom_dis = input

        para_s.value = nom_s
        para_dis.value = nom_dis
        
        for t in range(self.T):
            nom_st = nom_s[:, t:t+1]
            
            nom_phi = nom_st[2, 0]
            para_rot_list[t].value = np.array([[cos(nom_phi), -sin(nom_phi)],  [sin(nom_phi), cos(nom_phi)]])

    def assign_dual_parameter(self, LamMuZ_list):

        for index, LamMuZ in enumerate(LamMuZ_list):
            self.para_lam_list[index].value = LamMuZ[0]
            self.para_mu_list[index].value = LamMuZ[1]
            self.para_z_list[index].value = LamMuZ[2]


    def assign_obstacle_parameter(self, obstacle_list):
        
        # self.obstacle_template_list
        self.obstacle_num = len(obstacle_list)
        
        if self.obstacle_num < len(self.para_obstacle_list) and self.obstacle_num>0:
            last_element = obstacle_list[-1]
            obstacle_list += [last_element] * (len(self.para_obstacle_list) - self.obstacle_num)

        self.obstacle_num = len(obstacle_list)
        number = min(self.obstacle_num, self.max_obs_num)
        
        # for i, obs in enumerate(obstacle_list):
        for i in range(number):

            obs = obstacle_list[i]
            para_obs = self.para_obstacle_list[i]
            
            if isinstance(obs.A, list):

                obs_edge_num = obs.A[0].shape[0]
                
                for t in range(len(para_obs['A'])):
                    para_obs['A'][t].value[:obs_edge_num, :] = obs.A[t]
                    para_obs['b'][t].value[:obs_edge_num, :] = obs.b[t] 

                    para_obs['A'][t].value[obs_edge_num:, :] = 0
                    para_obs['b'][t].value[obs_edge_num:, :] = 0
                
                para_obs['cone_type'].value = np.array([1, 0]) if obs.cone_type == 'Rpositive' else np.array([0, 1])  # cone type: R_positive [0], norm2 [1]

            else:
                
                obs_edge_num = obs.A.shape[0]

                for t in range(len(para_obs['A'])):
                    
                    para_obs['A'][t].value[:obs_edge_num, :] = obs.A
                    para_obs['b'][t].value[:obs_edge_num, :] = obs.b

                    para_obs['A'][t].value[obs_edge_num:, :] = 0
                    para_obs['b'][t].value[obs_edge_num:, :] = 0
                
                para_obs['cone_type'].value = np.array([1, 0]) if obs.cone_type == 'Rpositive' else np.array([0, 1])  # cone type: R_positive [0], norm2 [1]
                    
                    
    def assign_combine_parameter_lamobs(self):
        
        for n in range(self.max_obs_num):

            para_lam_value = self.para_lam_list[n].value
            para_obs = self.para_obstacle_list[n]

            for t in range(self.T):
                lam = para_lam_value[:, t+1:t+2]
                obsA = para_obs['A'][t+1].value
                obsb = para_obs['b'][t+1].value

                self.para_obsA_lam_list[n].value[t+1, :] = lam.T @ obsA
                self.para_obsb_lam_list[n].value[t+1, :] = lam.T @ obsb
                    
    def assign_combine_parameter_stateobs(self):
        
        # self.para_obsA_lam_list = []   # lam.T @ obsA
        # self.para_obsb_lam_list = []   # lam.T @ obsb
        # self.para_obsA_rot_list = []   # obs.A @ rot
        # self.para_obsA_trans_list = []   # obs.A @ trans

        for n in range(self.max_obs_num):

            para_obs = self.para_obstacle_list[n]
 
            for t in range(self.T):
                obsA = para_obs['A'][t+1].value

                rot = self.para_rot_list[t].value
                trans = self.para_s.value[0:2, t+1:t+2]
                
                self.para_obsA_rot_list[n][t+1].value = obsA @ rot
                self.para_obsA_trans_list[n][t+1].value = obsA @ trans

        if self.obstacle_num == 0:

            for t in range(self.T):
                self.para_obsA_lam_list[n].value[t+1, :] = 0
                self.para_obsb_lam_list[n].value[t+1, :] = 0

    # endregion
    
    # region: solve the problem
    def iterative_solve(self, nom_s, nom_u, ref_states, ref_speed, obstacle_list, **kwargs):

        # obstacle_list: a list of obstacle instance
        #   obstacle: (A, b, cone_type)

        # start_time = time.time()
        
        self.para_ref_s.value = np.hstack(ref_states)[0:3, :]
        self.para_ref_speed.value = ref_speed

        # random.shuffle(obstacle_list)
        self.assign_state_parameter(nom_s, nom_u, self.para_dis.value)
        self.assign_obstacle_parameter(obstacle_list)

        iteration_time = time.time()
        for i in range(self.iter_num):

            start_time = time.time()
            opt_state_array, opt_velocity_array, resi_dual, resi_pri = self.rda_solver()
            if self.time_print: print('iteration ' + str(i) + ' time: ', time.time()-start_time)
            
            if resi_dual < self.iter_threshold and resi_pri < self.iter_threshold:
                if self.time_print: print('iteration early stop: '+ str(i))
                break
        
        if self.time_print:
            print('-----------------------------------------------')
            print('iteration time:', time.time() - iteration_time)
            print('==============================================')
        
        # info for debug
        opt_state_list = [state[:, np.newaxis] for state in opt_state_array.T ]
        info = {'ref_traj_list': ref_states, 'opt_state_list': opt_state_list}
        info['iteration_time'] = time.time() - start_time
        info['resi_dual'] = resi_dual
        info['resi_pri'] = resi_pri    
        
        return opt_velocity_array, info 

    def rda_solver(self):
        
        resi_dual, resi_pri = 0, 0
        
        # start_time = time.time()
        nom_s, nom_u, nom_dis = self.su_prob_solve()
        # print('- su problem solve:', time.time() - start_time)
        
        self.assign_state_parameter(nom_s, nom_u, nom_dis)
        # start_time = time.time()
        self.assign_combine_parameter_stateobs()
        # print('- other1:', time.time() - start_time)

        if self.obstacle_num != 0:
        # if self.max_obs_num != 0:
            # start_time = time.time()
            LamMuZ_list, resi_dual = self.LamMuZ_prob_solve()
            # print('- LamMu problem solve:', time.time() - start_time)

            self.assign_dual_parameter(LamMuZ_list)
            self.assign_combine_parameter_lamobs()

            resi_pri = self.update_xi()
            self.update_zeta()
            
        return nom_s, nom_u, resi_dual, resi_pri

    def update_zeta(self):

        for obs_index, para_obs in enumerate(self.para_obstacle_list):

            Im_list = []
            zeta = self.para_zeta_list[obs_index].value
            z = self.para_z_list[obs_index].value

            for t in range(self.T):

                lam_t = self.para_lam_list[obs_index].value[:, t+1:t+2]
                mu_t = self.para_mu_list[obs_index].value[:, t+1:t+2]
                z_t = self.para_z_list[obs_index].value[:, t:t+1]
                zeta_t = self.para_zeta_list[obs_index].value[:, t:t+1]

                trans_t = self.para_s.value[0:2, t+1:t+2]

                para_obsAt = para_obs['A'][t+1].value
                para_obsbt = para_obs['b'][t+1].value

                Im = lam_t.T @ para_obsAt @ trans_t - lam_t.T @ para_obsbt - mu_t.T @ self.car_tuple.h

                Im_list.append(Im)

            Im_array = np.hstack(Im_list)

            # temp= zeta + (Im_array - self.para_dis.value - z)  
            self.para_zeta_list[obs_index].value = zeta + (Im_array - self.para_dis.value - z)    
            
    def update_xi(self): 

        hm_list = []

        for obs_index, obs in enumerate(self.para_obstacle_list):
            for t in range(self.T):

                lam_t = self.para_lam_list[obs_index].value[:, t+1:t+2]
                mu_t = self.para_mu_list[obs_index].value[:, t+1:t+2]
                rot_t = self.para_rot_list[t].value
                xi_t = self.para_xi_list[obs_index].value[t+1:t+2, :]
                
                obsAt = obs['A'][t+1].value

                Hmt = mu_t.T @ self.car_tuple.G + lam_t.T @ obsAt @ rot_t
                self.para_xi_list[obs_index].value[t+1:t+2, :] = Hmt + xi_t    

                hm_list.append(Hmt)

        hm_array = np.vstack(hm_list)
        resi_pri = np.linalg.norm(hm_array)

        return resi_pri
    
    def su_prob_solve(self):
        self.prob_su.solve(solver=cp.ECOS, verbose=False)
        # self.prob_su.solve(solver=cp.SCS, verbose=False)

        if self.prob_su.status == cp.OPTIMAL or self.prob_su.status == cp.OPTIMAL_INACCURATE:
            return self.indep_s.value, self.indep_u.value, self.indep_dis.value
        else:
            print('No update of state and control vector')
            return self.para_s.value, self.para_u.value, self.para_dis.value

    def LamMuZ_prob_solve(self):
        
        input_args = []
        if self.process_num > 1:
            for obs_index in range(self.max_obs_num):

                nom_s = self.para_s.value
                nom_dis = self.para_dis.value
                nom_xi = self.para_xi_list[obs_index].value
                nom_zeta = self.para_zeta_list[obs_index].value
                receding = self.T
                nom_lam = self.para_lam_list[obs_index].value
                nom_mu = self.para_mu_list[obs_index].value
                nom_z = self.para_z_list[obs_index].value
                
                nom_obs_A = [o.value for o in self.para_obstacle_list[obs_index]['A']]    
                nom_obs_b = [o.value for o in self.para_obstacle_list[obs_index]['b']]   
                nom_obsA_rot = [p.value for p in self.para_obsA_rot_list[obs_index]]
                nom_obsA_trans = [p.value for p in self.para_obsA_trans_list[obs_index]] 
                nom_cone = self.para_obstacle_list[obs_index]['cone_type'].value

                input_args.append((obs_index, nom_s, nom_dis, nom_xi, receding, nom_lam, nom_mu, nom_z, nom_zeta, nom_obs_A, nom_obs_b, nom_obsA_rot, nom_obsA_trans, nom_cone))
            
            LamMuZ_list = pool.map(RDA_solver.solve_parallel, input_args)

        else:
            for obs_index in range(self.max_obs_num):
                prob = self.prob_LamMuZ_list[obs_index]
                input_args.append((prob, obs_index))
            
            LamMuZ_list = list(map(self.solve_direct, input_args))
        
        # update
        if len(LamMuZ_list) != 0:
            resi_dual_list = ([LamMu[3] for LamMu in LamMuZ_list])
            resi_dual = sum(resi_dual_list) / len(resi_dual_list)
        else:
            resi_dual = 0

        return LamMuZ_list, resi_dual

    def solve_parallel(input):
        
        obs_index, nom_s, nom_dis, nom_xi, receding, nom_lam, nom_mu, nom_z, nom_zeta, nom_obs_A, nom_obs_b, nom_obsA_rot, nom_obsA_trans, nom_cone = input
        
        prob = prob_LamMuZ_list[obs_index]

        # update parameter
        para_s.value = nom_s
        para_dis.value = nom_dis
        para_xi_list[obs_index].value = nom_xi
        para_zeta_list[obs_index].value = nom_zeta

        for t in range(receding):
            nom_st = nom_s[:, t:t+1]
            nom_phi = nom_st[2, 0]
            para_rot_list[t].value = np.array([[cos(nom_phi), -sin(nom_phi)],  [sin(nom_phi), cos(nom_phi)]])
            
            para_obsA_rot_list[obs_index][t+1].value = nom_obsA_rot[t+1]
            para_obsA_trans_list[obs_index][t+1].value = nom_obsA_trans[t+1]

            para_obstacle_list[obs_index]['A'][t+1].value = nom_obs_A[t+1]
            para_obstacle_list[obs_index]['b'][t+1].value = nom_obs_b[t+1]
            para_obstacle_list[obs_index]['cone_type'].value = nom_cone
        
        try:
            prob.solve(solver=cp.ECOS)
        except Exception as e:
            print('solve parallel error:', e, 'try another solver SCS')
            prob.solve(solver=cp.SCS)

        for variable in prob.variables():
            if 'lam_' in variable.name():
                indep_lam_value = variable.value
            elif 'mu_' in variable.name():
                indep_mu_value = variable.value
            elif 'z_' in variable.name():
                indep_z_value = variable.value
                
        if prob.status == cp.OPTIMAL:

            lam_diff = np.linalg.norm(indep_lam_value - nom_lam)
            mu_diff = np.linalg.norm(indep_mu_value - nom_mu)
            
            z_diff = np.linalg.norm(indep_z_value - nom_z)
            residual = lam_diff**2 + mu_diff**2 + z_diff**2

            return indep_lam_value, indep_mu_value, indep_z_value, residual

        else:
            print('Update Lam Mu Fail')
            return nom_lam, nom_mu, nom_z, inf

    def solve_direct(self, input):
        
        prob, obs_index = input

        try:
            prob.solve(solver=cp.ECOS)
        except Exception as e:
            print('solve parallel error:', e, 'try another solver SCS')
            prob.solve(solver=cp.SCS)

        # prob.solve(solver=cp.ECOS)
        # prob.solve(solver=cp.SCS)

        indep_lam = self.indep_lam_list[obs_index]
        indep_mu = self.indep_mu_list[obs_index]
        indep_z = self.indep_z_list[obs_index]

        para_lam = self.para_lam_list[obs_index]
        para_mu = self.para_mu_list[obs_index]
        para_z = self.para_z_list[obs_index]

        if prob.status == cp.OPTIMAL:

            lam_diff = np.linalg.norm(indep_lam.value - para_lam.value)
            mu_diff = np.linalg.norm(indep_mu.value - para_mu.value)
            z_diff = np.linalg.norm(indep_z.value - para_z.value)
            residual = lam_diff**2 + mu_diff**2 + z_diff**2

            return indep_lam.value, indep_mu.value, indep_z.value, residual
        else:
            print('Update Lam Mu Fail')
            return para_lam.value, para_mu.value, para_z.value, inf
        
    # endregion

    # region: formula， Hm, Im
    def Im_su(self, state, distance, para_lam, para_mu, para_z, para_zeta, para_obs, para_obsA_lam, para_obsb_lam):
        
        Im_list = []

        for t in range(self.T):
            # para_lam_t = para_lam[:, t+1:t+2]
            indep_trans_t = state[0:2, t+1:t+2]
            para_mu_t = para_mu[:, t+1:t+2]

            # para_obsAt = para_obs['A'][t+1]
            # para_obsbt = para_obs['b'][t+1]

            para_obsA_lam_t = para_obsA_lam[t+1:t+2, :]
            para_obsb_lam_t = para_obsb_lam[t+1:t+2, :]
             
            Im = para_obsA_lam_t @ indep_trans_t - para_obsb_lam_t - para_mu_t.T @ self.car_tuple.h
            Im_list.append(Im)

        Im_array = cp.hstack(Im_list)

        return Im_array[0, :] - distance[0, :] - para_z[0, :] + para_zeta[0, :]

    def Hm_su(self, rot, para_mu, para_lam, para_xi, para_obs, receding, para_obsA_lam):
        
        Hm_list = []

        for t in range(receding):
    
            lam_t = para_lam[:, t+1:t+2]
            mu_t = para_mu[:, t+1:t+2]
            para_xi_t = para_xi[t+1:t+2, :]
            indep_rot_t = rot[t]

            para_obsAt = para_obs['A'][t+1]

            para_obsA_lam_t = para_obsA_lam[t+1:t+2, :]

            Hmt = mu_t.T @ self.car_tuple.G + para_obsA_lam_t @ indep_rot_t + para_xi_t

            Hm_list.append(Hmt)

        return cp.vstack(Hm_list)

    def Hm_LamMu(self, indep_lam, indep_mu, para_rot_list, para_xi, para_obs, receding, para_obsA_rot):

        Hm_list = []
        for t in range(receding):
            indep_lam_t = indep_lam[:, t+1:t+2]
            indep_mu_t = indep_mu[:, t+1:t+2]
            
            para_rot_t = para_rot_list[t]
            para_xi_t = para_xi[t+1:t+2, :]

            para_obsA_rot_t = para_obsA_rot[t+1]
        
            Hmt = indep_mu_t.T @ self.car_tuple.G + indep_lam_t.T @ para_obsA_rot_t + para_xi_t
            Hm_list.append(Hmt)

        return cp.vstack(Hm_list)

    def Im_LamMu(self, indep_lam, indep_mu, indep_z, para_s, para_dis, para_zeta, para_obs, para_obsA_trans):

        # Im_array = cp.diag( indep_lam.T @ obs.A @ para_s[0:2] - indep_lam.T @ obs.b - indep_mu.T @ self.car_tuple.h ) 
        Im_list = []

        for t in range(self.T):
            indep_lam_t = indep_lam[:, t+1:t+2]
            indep_mu_t = indep_mu[:, t+1:t+2]
            para_obsbt = para_obs['b'][t+1]
            para_obsA_trans_t = para_obsA_trans[t+1]

            Im = indep_lam_t.T @ para_obsA_trans_t - indep_lam_t.T @ para_obsbt - indep_mu_t.T @ self.car_tuple.h
            Im_list.append(Im)

        Im_array = cp.hstack(Im_list)

        Im_lammu = Im_array[0, :] - para_dis[0, :] - indep_z[0, :] + para_zeta[0, :]

        return Im_lammu

    def dynamics_constraint(self, state, control_u, receding):

        temp_s1_list = []

        for t in range(receding):
            indep_st = state[:, t:t+1]
            indep_ut = control_u[:, t:t+1]

            ## dynamic constraints
            A = self.para_A_list[t]
            B = self.para_B_list[t]
            C = self.para_C_list[t]
            
            temp_s1_list.append(A @ indep_st + B @ indep_ut + C)
        
        constraints = [ state[:, 1:] == cp.hstack(temp_s1_list) ]

        return constraints
        
    def bound_su_constraints(self, state, control_u, para_s, max_speed, acce_bound):

        constraints = []

        constraints += [ cp.abs(control_u[:, 1:] - control_u[:, :-1] ) <= acce_bound ]  # constraints on speed accelerate
        constraints += [ cp.abs(control_u) <= max_speed]
        constraints += [ state[:, 0:1] == para_s[:, 0:1] ]

        return constraints

    def bound_dis_constraints(self, indep_dis):

        constraints = []

        constraints += [ cp.max(indep_dis) <= self.para_max_sd ] 
        constraints += [ cp.min(indep_dis) >= self.para_min_sd ]

        return constraints
    
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


    def linear_diff_model(self, nom_state, nom_u, dt):
        
        phi = nom_state[2, 0]
        v = nom_u[0, 0]

        A = np.array([ [1, 0, -v * dt * sin(phi)], [0, 1, v * dt * cos(phi)], [0, 0, 1] ])

        B = np.array([ [cos(phi)*dt, 0], [sin(phi)*dt, 0], 
                        [ 0, dt ] ])

        C = np.array([ [ phi*v*sin(phi)*dt ], [ -phi*v*cos(phi)*dt ], 
                        [ 0 ]])
                
        return A, B, C


    def linear_omni_model(self, nom_u, dt):
        
        phi = nom_u[1, 0]
        v = nom_u[0, 0]

        A = np.identity(3)
        B = np.array([ [ cos(phi) * dt, -v * sin(phi)* dt], [sin(phi)* dt, v*cos(phi) * dt], 
                        [ 0, 0 ] ])

        C = np.array([ [ phi*v*sin(phi)*dt ], [ -phi*v*cos(phi)*dt ], 
                        [ 0 ]])
        
        return A, B, C
    

    # def linear_omni_model(self, nom_u, dt):
        
    #     phi = nom_u[1, 0]
    #     v = nom_u[0, 0]

    #     A = np.identity(3)
    #     B = np.array([ [dt, 0], [0, dt], 
    #                     [ 0, 0 ] ])
        
    #     C = np.zeros((3, 1))
        
    #     return A, B, C
    

    def C0_cost(self, ref_s, ref_speed, state, control_u, ws, wu):

        if self.dynamics == 'omni':

            # temp = cp.norm(cp.norm(control_u, axis=0) - ref_speed)
            speed = control_u[0, :]
            diff_s = (state - ref_s)
            diff_u = (speed - ref_speed)
            # temp = cp.norm(control_u, axis=0)
            
            # return ws * cp.sum_squares(diff_s[0:2]) 
            return ws * cp.sum_squares(diff_s[0:2]) + wu*cp.sum_squares(diff_u) 
        else:
            speed = control_u[0, :]

            diff_s = (state - ref_s)
            diff_u = (speed - ref_speed)

            return ws * cp.sum_squares(diff_s) + wu*cp.sum_squares(diff_u) 

    def C1_cost(self, indep_dis, slack_gain):
        return ( -slack_gain * cp.sum(indep_dis) )

    def cone_cp_array(self, array, cone='Rpositive'):
        # cone for cvxpy: R_positive, norm2
        if cone == 'Rpositive':
            return cp.constraints.nonpos.NonPos(array)
        elif cone == 'norm2':
            return cp.constraints.nonpos.NonPos( cp.norm(array[0:-1], axis = 0) - array[-1]  )
    

    def cone_para_array(self, array, cone_flag):
        # cone for cvxpy: R_positive (0), norm2 (1); 
        
        norm_temp1 = (cp.norm(array[0:2], axis=0) - array[2])
        norm_temp2 = cp.reshape(norm_temp1, (1, self.T+1))

        proxy_array = cone_flag[0] * array + cone_flag[1] * norm_temp2

        return cp.constraints.nonpos.NonPos( proxy_array)

    # endregion


    def get_adjust_parameter(self):
        return {'slack_gain': self.para_slack_gain.value, 'max_sd': self.para_max_sd.value, 'min_sd': self.para_min_sd.value, 'ro1': self.ro1.value, 'ro2': self.ro2.value, 'ws': self.ws, 'wu': self.wu}



    def reset(self):
        
        for n in range(self.max_obs_num):
            for t in range(self.T):
                self.para_obsA_rot_list[n][t+1].value = np.zeros((self.max_edge_num, 2))  
                self.para_obsA_trans_list[n][t+1].value = np.zeros((self.max_edge_num, 1)) 

                self.para_obsA_lam_list[n].value[t+1, :] = 0
                self.para_obsb_lam_list[n].value[t+1, :] = 0
       
    


    


    


    

