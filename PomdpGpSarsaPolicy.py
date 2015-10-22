__author__ = 'aishwarya'

# This implements the episodic GP-SARSA algorithm as in 
#       Gaussian Processes for POMDP-Based Dialogue Manager Optimization
#       - Milica Gasic & Steve young, TASLP 2014
#
# As far as possible, the variable names follow the notation of the paper
# but bars are ignored for example, \overline{x} will just be called x.
# Also, x^-1 will be called x_inv and x^T will be called x_trans
#
# Implementation has been done using numpy for increased readability and
# in the hope that it scales better. I always use numpy matrices not 
# arrays because arrays don't do vector math in an intuitive way

import numpy as np, sys, math
from Utils import *

class PomdpGpSarsaPolicy :
    def __init__(self, knowledge, load_from_file=False) :
        self.knowledge = knowledge
        self.gamma = self.knowledge.gamma
        self.sigma = self.knowledge.gp_sarsa_std_dev
        
        if load_from_file :
            self.mu = load_model('mu')
            self.C = load_model('C')
            self.c = load_model('c')
            self.d = load_model('d')
            self.v = load_model('v')
            self.D = load_model('D')
            self.g = load_model('g')
            self.K_inv = load_model('K_inv')
            self.first_episode = False
        else :
            self.mu = np.matrix([[]])
            self.C = np.matrix([[]])
            self.c = np.matrix([[]])
            self.d = 0.0
            self.v = float('inf')
            self.D = list()
            self.K_inv = np.matrix([[]])
            self.first_episode = True
        self.first_turn = True
        self.a = None
        self.b = None
        self.g = np.matrix([[]])
        self.delta = 0
        self.nu = self.knowledge.sparsification_param
        self.k_b_a = np.matrix([[]])
    
    # b and b_prime are summary states
    def calc_k(self, (b, a), (b_prime, a_prime)) :
        action_kernel_value = float(a == a_prime)
        state_kernel_value = b.calc_kernel(b_prime)
        return state_kernel_value * action_kernel_value
    
    def calc_k_vector(self, b, a) :
        replicas = [(b, a)] * len(self.D)
        return np.matrix([map(self.calc_k, replicas, self.D)]).T
    
    def get_closest_dictionary_point(self, target_b, target_a) :
        min_dist = sys.maxint
        min_dist_point = None
        min_idx = None
        for (idx, (b, a)) in enumerate(self.D) :
            dist = self.calc_k((target_b, target_a), (b, a))
            if dist < min_dist :
                min_dist = dist
                min_dist_point = (b, a)
                min_idx = idx
        return (min_idx, min_dist_point)
    
    def get_random_action(self) :
        num_actions = len(self.knowledge.summary_system_actions)
        sample = int(np.random.uniform(0, num_actions))
        return self.knowledge.summary_system_actions[sample]
    
    def pi(self, b) :
        if self.D is None or len(self.D) == 0 :
            return self.get_random_action()
        max_q_val = -sys.maxint
        best_action = None
        for a in self.knowledge.summary_system_actions :
            (idx, closest_dict_point) = self.get_closest_dictionary_point(b, a)
            mean = self.mu[idx]
            cov = self.C[idx][idx]
            q = np.random.normal(mean, cov)
            if q > max_q_val :
                max_q_val = q
                best_action = a
        return best_action
    
    def get_initial_action(self, initial_state) :
        self.b = initial_state
        
        if self.first_episode :
            self.first_episode = False
            self.a = self.get_random_action()
            self.D = [(self.b, self.a)]
            self.mu = self.get_zero_vector(1)
            self.C = self.get_zero_vector(1)
            self.c = self.get_zero_vector(1)
            
            # K^{-1} = 1 / k((b, a), (b, a))
            self.K_inv = np.matrix([[1.0 / self.calc_k((self.b, self.a), (self.b, self.a))]])
        else :
            self.a = pi(self.b)
        
        self.c = self.get_zero_vector(len(self.D))
        self.d = 0.0
        self.v = float('inf')
        self.k_b_a = self.calc_k_vector(self.b, self.a)
        
        # g = K^{-1}k(b, a)
        self.g = self.K_inv * self.k_b_a
        
        # delta = k((b, a), (b, a)) - (k(b, a)^T * g)
        self.delta = self.calc_k((self.b, self.a), (self.b, self.a)) - (self.k_b_a.T * self.g).item(0,0)
        
        if self.delta > self.nu :
            # This should ideally happen only if the point is not already
            # in D so it is fine to append to D without checking
            self.D.append((self.b, self.a))
            
            # K^{-1} = (1/delta) * [delta * K^{-1} + gg^T   -g ]
            #                      [    -g^T                 1 ]
            self.K_inv = (self.delta * self.K_inv) + (self.g * self.g.T)
            self.K_inv = np.append(self.K_inv, -self.g, 1)
            new_row = np.append(-self.g.T, np.matrix([[1]]), 1)
            self.K_inv = np.append(self.K_inv, new_row, 0)
            self.K_inv = (1.0 / self.delta) * self.K_inv 
            
            # g = [0, 0, ... 1]^T
            self.g = get_zero_vector(len(D))
            self.g[0, len(D) - 1] = 1
            
            # mu = [ mu ]
            #      [ 0  ] 
            self.mu = np.append(self.mu, np.matrix([[0]]), 0)
            
            # C = [  C   0 ]
            #     [ 0^T  0 ]
            orig_C_size = len(self.C)
            self.C = np.append(self.C, get_zero_vector(orig_C_size), 1)
            self.C = np.append(self.C, get_zero_vector(orig_C_size + 1), 0)
            
            # c = [ c ]
            #      [ 0  ] 
            self.c = np.append(self.c, np.matrix([[0]]), 0)
        return self.a
    
    def get_next_action(reward, current_state) :
        b_prime = current_state
        r_prime = reward
        
        a_prime = self.pi(self.b_prime)
        k_b_prime_a_prime = calc_k_vector(self.b_prime, self.a_prime)
        
        # g' = K^{-1}k(b',a')
        g_prime = self.K_inv * self.k_b_prime_a_prime
        
        # delta = k((b',a'), (b',a')) - (k(b',a')^T * g)
        k = self.calc_k((self.b_prime, self.a_prime), (self.b_prime, self.a_prime))
        self.delta = k - (k_vector.T * self.g_prime).item(0,0)
        
        delta_k = self.k_b_a - self.gamma * self.k_b_prime_a_prime
        
        self.d = (self.gamma * self.sigma * self.sigma / self.v) * d + self.r_prime - (self.delta_k.T * self.mu).item(0,0)
    
        if self.delta > self.nu :
            self.D.append((self.b_prime, self.a_prime))
            
            # K^{-1} = (1/delta) * [delta * K^{-1} + gg^T   -g ]
            #                      [    -g^T                 1 ]
            self.K_inv = (self.delta * self.K_inv) + (self.g * self.g.T)
            self.K_inv = np.append(self.K_inv, -self.g, 1)
            new_row = np.append(-self.g.T, np.matrix([[1]]), 1)
            self.K_inv = np.append(self.K_inv, new_row, 0)
            self.K_inv = (1.0 / self.delta) * self.K_inv 
            
            # g' = [0, 0, ... 1]^T
            g_prime = get_zero_vector(len(D))
            g_prime[0, len(D) - 1] = 1
            
            h = np.append(self.g, np.matrix([[-gamma]]), 0)
            
            # delta_k_tt = g^T(k(b,a) - 2*gamma*k(b',a')) + (gamma^2)k((b',a'),(b',a'))
            delta_k_tt = (self.g * (self.k_b_a - 2 * gamma * k_b_prime_a_prime)).item(0,0) + gamma * gamma * k
            
            # c' = (gamma * sigma^2 / v)[ c ]  + h - [ C * delta_k ]
            #                           [ 0 ]        [     0       ]
            c_prime = ((self.gamma * self.sigma * self.sigma / self.v) * np.append(self.c, np.matrix([[0]]), 0)) + h - np.append(self.C * delta_k, np.matrix([[0]]), 0)
        
            # v = (1 + gamma^2)sigma^2 + delta_k_tt - (delta_k^T)(C)(delta_k) + (2*gamma*sigma^2/v)(c^T delta_k) - (gamma^2 * sigma^4 / v)
            self.v = (1 + self.gamma * self.gamma) * self.sigma * self.sigma + delta_k_tt - (delta_k.T * self.C * delta_k) + (2 * self.gamma * self.sigma * self.sigma / self.v) * (self.c.T * delta_k).item(0,0) - ((self.gamma ** 2) * (self.sigma ** 4) / self.v)
            
            # mu = [ mu ]
            #      [ 0  ] 
            self.mu = np.append(self.mu, np.matrix([[0]]), 0)
            
            # C = [  C   0 ]
            #     [ 0^T  0 ]
            orig_C_size = len(self.C)
            self.C = np.append(self.C, get_zero_vector(orig_C_size), 1)
            self.C = np.append(self.C, get_zero_vector(orig_C_size + 1), 0)
        
        else :
            h = g - self.gamma * g_prime
            c_prime = ((self.gamma * self.sigma * self.sigma / self.v) * self.c + h - self.C * delta_k
            
            self.v = (1 + self.gamma * self.gamma) * self.sigma * self.sigma + (delta_k.T * (c_prime + (self.gamma * self.sigma * self.sigma / self.v) * self.c)).item(0,0) - ((self.gamma ** 2) * (self.sigma ** 4) / self.v)
        
        self.mu = self.mu + self.c * (d/v)
        self.C = self.C + (1.0 / v) * (self.c * self.c.T)
        
        self.c = c_prime
        self.g = g_prime
        self.b = b_prime
        self.a = a_prime
        
        # TODO: Check this. You might need to calculate it again if D has changed
        self.k_b_a = k_b_prime_a_prime
        
        return self.a
        
    def update_final_reward(reward) :
        r_prime = reward
        g_prime = get_zero_vector(len(self.D))
        self.delta = 0
        delta_k = self.k_b_a

        self.d = (self.gamma * self.sigma * self.sigma / self.v) * d + self.r_prime - (self.delta_k.T * self.mu).item(0,0)
    
        h = g - self.gamma * g_prime
        c_prime = ((self.gamma * self.sigma * self.sigma / self.v) * self.c + h - self.C * delta_k
        
        self.v = self.sigma * self.sigma + (delta_k.T * (c_prime + (self.gamma * self.sigma * self.sigma / self.v) * self.c)).item(0,0) - ((self.gamma ** 2) * (self.sigma ** 4) / self.v)
        
        self.mu = self.mu + self.c * (d/v)
        self.C = self.C + (1.0 / v) * (self.c * self.c.T)
        
        self.c = c_prime
        self.g = g_prime
        
        self.save_vars()
        
    # A util to get a zero vector because the command is non-intuitive        
    def get_zero_vector(self, size) :
        return np.matrix(np.zeros(size)).T    
            
    def save_vars(self) :
        save_model(self.mu, 'mu')
        save_model(self.C, 'C')
        save_model(self.c, 'c')
        save_model(self.d, 'd')
        save_model(self.v, 'v')
        save_model(self.D, 'D')
        save_model(self.g, 'g')
        save_model(self.K_inv, 'K_inv')

    def print_vars(self) :
        print 'self.mu = ', self.mu
        print 'self.C = ', self.C
        print 'self.c = ', self.c
        print 'self.d = ', self.d
        print 'self.v = ', self.v
        print 'self.D = ', self.D
        print 'self.K_inv = ', self.K_inv
        print 'self.a = ', self.a
        print 'self.b = ', self.b
        print 'self.g = ', self.g
        print 'self.delta = ', self.delta
