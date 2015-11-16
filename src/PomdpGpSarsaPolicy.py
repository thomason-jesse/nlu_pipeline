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

# TODO: Make candidate action sets which are sensible

import numpy as np, sys, math, copy, itertools
from Utils import *
from SystemAction import SystemAction
from Action import Action
from SummaryState import SummaryState
from Partition import Partition
from Utterance import Utterance

class PomdpGpSarsaPolicy :
    def __init__(self, knowledge, load_from_file=False) :
        self.knowledge = knowledge
        self.gamma = self.knowledge.gamma
        self.sigma = self.knowledge.gp_sarsa_std_dev
        self.dist_threshold = 0.2
        
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
        self.a = None
        self.b = None
        self.g = np.matrix([[]])
        self.delta = 0
        self.nu = self.knowledge.sparsification_param
        self.k_b_a = np.matrix([[]])
    
    # b and b_prime are summary states
    def calc_k(self, (b, a), (b_prime, a_prime)) :
        #print 'b = ', b.get_feature_vector()
        #print 'b\' = ', b_prime.get_feature_vector()
        #print 'a = ', a, 'a_prime = ', a_prime
        
        action_kernel_value = 3 * float(a == a_prime)
        #print 'action_kernel_value = ', action_kernel_value
        state_kernel_value = b.calc_kernel(b_prime)
        #print 'state_kernel_value = ', state_kernel_value
        return state_kernel_value * action_kernel_value
    
    def calc_k_vector(self, b, a) :
        replicas = [(b, a)] * len(self.D)
        return np.matrix([map(self.calc_k, replicas, self.D)]).T
    
    def get_closest_dictionary_point(self, target_b, target_a) :
        max_sim = -sys.maxint
        max_sim_point = None
        max_idx = None
        for (idx, (b, a)) in enumerate(self.D) :
            sim = self.calc_k((target_b, target_a), (b, a))
            if sim > max_sim :
                max_sim = sim
                max_sim_point = (b, a)
                max_idx = idx
        return (max_idx, max_sim_point)
    
    def get_random_action(self, current_state) :
        candidate_actions = self.get_candidate_actions(current_state)
        num_actions = len(candidate_actions)
        sample = int(np.random.uniform(0, num_actions))
        return candidate_actions[sample]
    
    def get_candidate_actions(self, current_state) :
        if current_state is None or current_state.top_hypothesis is None :
            return self.knowledge.summary_system_actions
        elif current_state.top_hypothesis[0] is None or current_state.top_hypothesis[0].possible_goals is None :
            return self.knowledge.summary_system_actions
        elif len(current_state.top_hypothesis[0].possible_goals) != 1 :
            return self.knowledge.summary_system_actions
        else :
            unique_goal = current_state.top_hypothesis[0].possible_goals[0]
            candidate_actions = copy.deepcopy(self.knowledge.summary_system_actions)
            if unique_goal == 'speak_t' or unique_goal == 'speak_e' :
                candidate_actions.remove('request_missing_param')
            return candidate_actions
    
    def pi(self, b) :
        if self.D is None or len(self.D) == 0 :
            #return self.get_random_action(b)
            return self.get_action_from_hand_coded_policy(b)
            
        max_q_val = -sys.maxint
        best_action = None
        candidate_actions = self.get_candidate_actions(b)
        for a in candidate_actions :
            (idx, closest_dict_point) = self.get_closest_dictionary_point(b, a)
            
            sim_to_dict_point = self.calc_k((b, a), closest_dict_point)
            #print 'sim_to_dict_point = ', sim_to_dict_point
            #print 'closest_dict_point : ', closest_dict_point[0].get_feature_vector(), ', action = ', closest_dict_point[1] 
            if sim_to_dict_point < self.dist_threshold :
                print 's = ', b.get_feature_vector()
                print 'a = ', a
                print 'Using hand coded policy because sim = ', sim_to_dict_point
                hand_coded_policy_a = self.get_action_from_hand_coded_policy(b)
                #print 'hand_coded_policy_a = ', hand_coded_policy_a
                if a == hand_coded_policy_a :
                    mean = 1
                    std_dev = 1
                else :
                    mean = 0
                    std_dev = 1
            else :
                mean = self.mu[idx, 0]
                #std_dev = math.sqrt(self.C[idx, idx])
                std_dev = math.sqrt(abs(self.C[idx, idx]))
            
            if std_dev < 0.0000001 :
                q = mean
            else :
                q = np.random.normal(mean, std_dev)
            if q > max_q_val :
                max_q_val = q
                best_action = a
            print 'action = ', a, ', mean = ', mean, ', std_dev = ', std_dev, ', q = ', q, '\n'
        return best_action
    
    def get_initial_action(self, initial_state) :
        #print '\nIn get_initial_action'     # DEBUG
        #self.print_vars()                   # DEBUG
        
        self.b = initial_state
        
        # Forcing the initial action to be repeat_goal
        # Not sure if thsi is correct from the RL perspective
        self.a = 'repeat_goal'
        
        if self.first_episode :
            self.first_episode = False
            self.D = [(self.b, self.a)]
            self.mu = self.get_zero_vector(1)
            self.C = self.get_zero_vector(1)
            
            # K^{-1} = 1 / k((b, a), (b, a))
            self.K_inv = np.matrix([[1.0 / self.calc_k((self.b, self.a), (self.b, self.a))]])
        
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
            self.g = self.get_zero_vector(len(self.D))
            self.g[len(self.D) - 1, 0] = 1
            
            # mu = [ mu ]
            #      [ 0  ] 
            self.mu = np.append(self.mu, np.matrix([[0]]), 0)
            
            # C = [  C   0 ]
            #     [ 0^T  0 ]
            orig_C_size = len(self.C)
            self.C = np.append(self.C, self.get_zero_vector(orig_C_size), 1)
            self.C = np.append(self.C, self.get_zero_vector(orig_C_size + 1).T, 0)
            
            # c = [ c ]
            #      [ 0  ] 
            self.c = np.append(self.c, np.matrix([[0]]), 0)
        
        #print 'End of get_initial_action'   # DEBUG   
        #self.print_vars()                   # DEBUG
        #print '-------------------------'   # DEBUG
        return (self.a, self.get_system_action_requirements(self.a, self.b))
    
    def get_next_action(self, reward, current_state) :
        #print '\nIn get_next_action'        # DEBUG
        #self.print_vars()                   # DEBUG
        
        b_prime = current_state
        r_prime = reward
        
        a_prime = self.pi(b_prime)
        k_b_prime_a_prime = self.calc_k_vector(b_prime, a_prime)
        
        # g' = K^{-1}k(b',a')
        g_prime = self.K_inv * k_b_prime_a_prime
        
        # delta = k((b',a'), (b',a')) - (k(b',a')^T * g)
        k = self.calc_k((b_prime, a_prime), (b_prime, a_prime))
        self.delta = k - (k_b_prime_a_prime.T * g_prime).item(0,0)
        
        delta_k = self.k_b_a - self.gamma * k_b_prime_a_prime
        
        self.d = (self.gamma * self.sigma * self.sigma / self.v) * self.d + r_prime - (delta_k.T * self.mu).item(0,0)
    
        if self.delta > self.nu :
            self.D.append((b_prime, a_prime))
            
            # K^{-1} = (1/delta) * [delta * K^{-1} + gg^T   -g ]
            #                      [    -g^T                 1 ]
            self.K_inv = (self.delta * self.K_inv) + (self.g * self.g.T)
            self.K_inv = np.append(self.K_inv, -self.g, 1)
            new_row = np.append(-self.g.T, np.matrix([[1]]), 1)
            self.K_inv = np.append(self.K_inv, new_row, 0)
            self.K_inv = (1.0 / self.delta) * self.K_inv 
            
            # g' = [0, 0, ... 1]^T
            g_prime = self.get_zero_vector(len(self.D))
            g_prime[len(self.D) - 1, 0] = 1
            
            h = np.append(self.g, np.matrix([[-self.gamma]]), 0)
            
            # delta_k_tt = g^T(k(b,a) - 2*gamma*k(b',a')) + (gamma^2)k((b',a'),(b',a'))
            delta_k_tt = (self.g.T * (self.k_b_a - 2 * self.gamma * k_b_prime_a_prime)).item(0,0) + self.gamma * self.gamma * k
            
            # c' = (gamma * sigma^2 / v)[ c ]  + h - [ C * delta_k ]
            #                           [ 0 ]        [     0       ]
            c_prime = ((self.gamma * self.sigma * self.sigma / self.v) * np.append(self.c, np.matrix([[0]]), 0)) + h - np.append(self.C * delta_k, np.matrix([[0]]), 0)
        
            # v = (1 + gamma^2)sigma^2 + delta_k_tt - (delta_k^T)(C)(delta_k) + (2*gamma*sigma^2/v)(c^T delta_k) - (gamma^2 * sigma^4 / v)
            self.v = (1 + self.gamma * self.gamma) * self.sigma * self.sigma + delta_k_tt - (delta_k.T * self.C * delta_k).item(0,0) + (2 * self.gamma * self.sigma * self.sigma / self.v) * (self.c.T * delta_k).item(0,0) - ((self.gamma ** 2) * (self.sigma ** 4) / self.v)
            
            # mu = [ mu ]
            #      [ 0  ] 
            self.mu = np.append(self.mu, np.matrix([[0]]), 0)
            
            # C = [  C   0 ]
            #     [ 0^T  0 ]
            orig_C_size = len(self.C)
            self.C = np.append(self.C, self.get_zero_vector(orig_C_size), 1)
            self.C = np.append(self.C, self.get_zero_vector(orig_C_size + 1).T, 0)
        
        else :
            h = self.g - self.gamma * g_prime
            c_prime = (self.gamma * self.sigma * self.sigma / self.v) * self.c + h - self.C * delta_k
            #print 'self.v = ', self.v
            self.v = (1 + self.gamma * self.gamma) * self.sigma * self.sigma + (delta_k.T * (c_prime + (self.gamma * self.sigma * self.sigma / self.v) * self.c)).item(0,0) - ((self.gamma ** 2) * (self.sigma ** 4) / self.v)
            #print 'self.v = ', self.v
        
        if self.c.shape != self.mu.shape :        
            self.c = np.append(self.c, np.matrix([[0]]), 0)
        self.mu = self.mu + self.c * (self.d / self.v)
        self.C = self.C + (1.0 / self.v) * (self.c * self.c.T)

        self.c = c_prime
        self.g = g_prime
        self.b = b_prime
        self.a = a_prime
        
        if self.delta > self.nu :
            self.k_b_a = self.calc_k_vector(self.b, self.a)
        else :
            self.k_b_a = k_b_prime_a_prime
        
        #print 'End of get_next_action'      # DEBUG   
        #self.print_vars()                   # DEBUG
        #print '-------------------------'   # DEBUG
        
        return (self.a, self.get_system_action_requirements(self.a, self.b))
        
    def update_final_reward(self, reward) :
        #print '\nIn update_final_reward'    # DEBUG
        #self.print_vars()                   # DEBUG
        
        r_prime = reward
        g_prime = self.get_zero_vector(len(self.D))
        self.delta = 0
        self.k_b_a = self.calc_k_vector(self.b, self.a)
        delta_k = self.k_b_a

        self.d = (self.gamma * self.sigma * self.sigma / self.v) * self.d + r_prime - (delta_k.T * self.mu).item(0,0)
    
        h = self.g - self.gamma * g_prime
        c_prime = (self.gamma * self.sigma * self.sigma / self.v) * self.c + h - self.C * delta_k
        
        self.v = self.sigma * self.sigma + (delta_k.T * (c_prime + (self.gamma * self.sigma * self.sigma / self.v) * self.c)).item(0,0) - ((self.gamma ** 2) * (self.sigma ** 4) / self.v)
        
        self.mu = self.mu + self.c * (self.d / self.v)
        self.C = self.C + (1.0 / self.v) * (self.c * self.c.T)
        
        self.c = c_prime
        self.g = g_prime
        
        self.save_vars()

        #print 'End of update_final_reward'  # DEBUG   
        #self.print_vars()                   # DEBUG
        #print '-------------------------'   # DEBUG

    # A util to get a zero vector because the command is non-intuitive        
    def get_zero_vector(self, size) :
        return np.matrix(np.zeros(size)).T 
        
    # Converts a state to a single Action object if possible
    # If the partition either allows more than one goal or more than one
    # value for any param that action needs, then return None
    def resolve_state_to_goal(self, state) :
        if state is None or state.top_hypothesis is None :
            return None
        partition = state.top_hypothesis[0]   
        if partition.possible_goals is None or len(partition.possible_goals) != 1 :
            return None
        goal = partition.possible_goals[0]
        action = Action(goal)
        param_order = state.knowledge.param_order[goal]
        params = []
        if partition.possible_param_values is None :
            return None
        for param_name in param_order :
            if param_name not in partition.possible_param_values or len(partition.possible_param_values[param_name]) != 1 :
                return None
            else :
                params.append(partition.possible_param_values[param_name][0])
        action.params = params    
        return action
    
    def get_system_action_requirements(self, action_type, state) :
        if action_type == 'repeat_goal' :
            return SystemAction(action_type)
        elif action_type == 'take_action' :
            return self.resolve_state_to_goal(state)
            
        elif action_type == 'confirm_action' :
            if state.top_hypothesis is None :
                goal_idx = int(np.random.uniform(0, len(state.knowledge.goal_actions)))
                return SystemAction(action_type, state.knowledge.goal_actions[goal_idx])
            goal = state.top_hypothesis[0].possible_goals[0]
            system_action = SystemAction(action_type, goal)
            param_order = state.knowledge.param_order[goal]
            params = dict()
            partition_params = state.top_hypothesis[0].possible_param_values
            if partition_params is None :
                return system_action
            for param_name in param_order :
                if param_name in partition_params and len(partition_params[param_name]) == 1 :
                    params[param_name] = partition_params[param_name][0]
                else :
                    print 'Uncertain about ', param_name, ' : ', partition_params[param_name]
            system_action.referring_params = params
            return system_action
            
        elif action_type == 'request_missing_param' :
            if state.top_hypothesis is None :
                goal = None
                system_action = SystemAction(action_type)
                param_idx = int(np.random.uniform(0, len(state.knowledge.goal_params)))
                system_action.extra_data = [state.knowledge.goal_params[param_idx]]
                return system_action
                    
            goal = state.top_hypothesis[0].possible_goals[0]
            system_action = SystemAction(action_type, goal)
            param_order = state.knowledge.param_order[goal]
            params = dict()
            param_to_request = None
            partition_params = state.top_hypothesis[0].possible_param_values
            if partition_params is None :
                return system_action
            for param_name in param_order :
                if param_name not in partition_params or len(partition_params) != 1 :
                    if param_to_request is None :
                        param_to_request = param_name
                else :
                    params[param_name] = partition_params[param_name][0]
            system_action.referring_params = params
            system_action.extra_data = [param_to_request]
            
            if param_to_request is None :
                # The top hypothesis partition doesn't have uncertain 
                # params but it is possible it is not of high enough 
                # confidence
                
                # If there is no second hypothesis, just confirm any value
                # This param si chosen at random so that you don't get 
                # stuck in a loop here
                if state.second_hypothesis is None or state.second_hypothesis.partition.possible_param_values is None :
                    param_idx = int(np.random.uniform(0, len(param_order)))
                    system_action.extra_data = [param_order[param_idx]]
                    return system_action
                
                # A good heuristic is to see in what params the first 
                # and second hypotheses differ. Any one of these is 
                # likely to help. 
                second_params = state.second_hypothesis.partition.possible_param_values
                for param_name in param_order :
                    top_param_value = partition_params[param_name][0]
                    if param_name not in second_params or top_param_value not in second_params[param_name] or len(second_params[param_name]) != 1 :
                        system_action.extra_data = [param_name]
                        return system_action
                
                # If you reached here, this is probably an inappropriate 
                # action so just verify a random param. 
                param_idx = int(np.random.uniform(0, len(param_order)))
                system_action.extra_data = [param_order[param_idx]]
                return system_action
                
            return system_action
                    
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
       
    def get_action_from_hand_coded_policy(self, state) :
        if state.num_dialog_turns > 10 :
            return 'take_action'
        feature_vector = state.get_feature_vector()
        num_goals = feature_vector[2]
        num_uncertain_params = feature_vector[3]
        if num_goals == 1 :
            if num_uncertain_params == 0 :
                if state.top_hypothesis_prob < 0.3 :
                    return 'request_missing_param'
                elif state.top_hypothesis_prob < 0.9 :
                    return 'confirm_action'
                else :
                    return 'take_action'
            else :
                return 'request_missing_param'
        else :
            return 'repeat_goal'      
        
    def create_initial_policy(self) :
        probs = [x * 0.1 for x in xrange(0, 10)]
        num_goals = range(0, len(self.knowledge.goal_actions))
        num_uncertain_params = range(0, len(self.knowledge.goal_params)) + [sys.maxint]
        num_dialog_turns = range(0, 10)
        yes_no = ['yes', 'no']
        utterance_type = [None, 'inform', 'affirm', 'deny']
        values = list(itertools.product(*[probs, probs, num_goals, num_uncertain_params, num_dialog_turns, yes_no, utterance_type]))

        # Warning: Make sure you have a default value for every param in 
        # Knowledge.goal_params
        default_param_values = dict()
        default_param_values['patient'] = ['ray']
        default_param_values['recipient'] = ['peter']
        default_param_values['location'] = ['l3_512']

        examples = list()
        for (top_prob, sec_prob, num_goals, num_uncertain_params, num_dialog_turns, match, utterance_type) in values :
            if num_goals != 1 and num_uncertain_params != sys.maxint :
                continue
            elif num_goals == 1 and num_uncertain_params == sys.maxint :
                continue
            elif num_goals != len(self.knowledge.goal_actions) and utterance_type is None :
                continue
            s = SummaryState()
            s.knowledge = self.knowledge
            s.top_hypothesis_prob = top_prob
            s.second_hypothesis_prob = sec_prob
            s.num_dialog_turns = num_dialog_turns
        
            if num_goals != 1 :
                if utterance_type is not None :
                    utterance = Utterance(utterance_type)
                    s.top_hypothesis = (Partition(self.knowledge.goal_actions[0:num_goals]), utterance)
                    if match == 'yes' :
                        s.second_hypothesis = (Partition(self.knowledge.goal_actions[0:num_goals]), utterance)
            
                if s.get_feature_vector() != [top_prob, sec_prob, num_goals, num_uncertain_params, num_dialog_turns, match, utterance_type] :
                    print 'Problem!'
                    print s.get_feature_vector()
                    print (top_prob, sec_prob, num_goals, num_uncertain_params, num_dialog_turns, match, utterance_type)
                    print '\n'
                    
                examples.append((s, 'repeat_goal'))
            else :
                #for goal in self.knowledge.goal_actions :
                goal = 'remind'
                params = dict()
                
                param_order = self.knowledge.param_order[goal]
                if num_uncertain_params > len(param_order) :
                    continue
                num_certain_params = len(param_order) - num_uncertain_params  
                #print 'len(param_order) = ', len(param_order)
                #print 'num_uncertain_params = ', num_uncertain_params          
                #print 'num_certain_params = ', num_certain_params
                for (idx, param_name) in enumerate(param_order) :
                    if idx < num_certain_params :
                        params[param_name] = default_param_values[param_name]
                    else :
                        params[param_name] = self.knowledge.goal_params_values
                
                s = SummaryState()
                s.knowledge = self.knowledge
                s.top_hypothesis_prob = top_prob
                s.second_hypothesis_prob = sec_prob
                s.num_dialog_turns = num_dialog_turns        
                
                if utterance_type is not None :
                    utterance = Utterance(utterance_type)
                    s.top_hypothesis = (Partition([goal], params), utterance)
                    if match == 'yes' :
                        s.second_hypothesis = (Partition([goal], params), utterance)                    
            
                if s.get_feature_vector() != [top_prob, sec_prob, num_goals, num_uncertain_params, num_dialog_turns, match, utterance_type] :
                    print 'Problem!'
                    print s.get_feature_vector()
                    print (top_prob, sec_prob, num_goals, num_uncertain_params, num_dialog_turns, match, utterance_type)
                    print s.top_hypothesis[0].possible_param_values
                    print '\n'
                                
                if num_uncertain_params == 0 :
                    if top_prob < 0.3 :
                        action = 'request_missing_param'
                    elif top_prob < 0.9 :
                        action = 'confirm_action'
                    else :
                        action = 'take_action'
                else :
                    if num_uncertain_params > 0 :
                        action = 'request_missing_param'
                examples.append((s, action))
                    
        print len(examples), 'examples'        
        D = list()
        mean = []
        cov = []
        actions = self.knowledge.summary_system_actions
        for (b, a) in examples :
            for a_prime in actions :
                D.append((b, a_prime))
                if a == a_prime :
                    mean.append(1.0)
                else :
                    mean.append(0.0)
        print 'len(D) = ', len(D)
        cov = numpy.matrix(numpy.zeros((len(D), len(D))))
        for i in range(0, len(D)) :
            cov[(i,i)] = 0.1
        self.D = D
        self.mu = numpy.matrix([[x] for x in mean])
        self.C = cov
        
        # TODO: Calculate K^-1
                    
        
    
        
