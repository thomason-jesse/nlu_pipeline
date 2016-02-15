__author__ = 'aishwarya'

# This implements the KTD-Q algorithm used in 
#       Daubigney, L., Geist, M., Chandramohan, S. & Pietquin, O. (2012), 
#       "A Comprehensive Reinforcement Learning Framework for Dialogue 
#        Management Optimisation", IEEE Journal of Selected Topics in 
#        Signal Processing., December, 2012, Vol. 6(8), pp. 891-902
#
# The algorithm is designed in 
#       Geist, M. & Pietquin, O. (2010), "Kalman Temporal Differences", 
#       Journal of Artificial Intelligence Research (JAIR)., October, 
#       2010, Vol. 39, pp. 483-532.
#
# Variable names imitate the paper but drop superscripts and subscripts 
# when unnecessary for the sake of readibility
# TODO: Include a README with complete algorithm with notation 
# corresponding variable names to code
#
# Implementation has been done using numpy for increased readability and
# in the hope that it scales better. I always use numpy matrices not 
# arrays because arrays don't do vector math in an intuitive way

import numpy as np, sys, math, copy, itertools
from Utils import *
from SystemAction import SystemAction
from Action import Action
from SummaryState import SummaryState
from Partition import Partition
from Utterance import Utterance

class PomdpKtdqPolicy :
    def __init__(self, knowledge, load_from_file=False) :
        self.knowledge = knowledge
        
        # Calculate number of features
        self.n = 3
        self.n += len(self.knowledge.ktdq_rbf_centres)
        self.n += len(self.knowledge.goal_actions)
        self.n += len(self.knowledge.goal_params) + 1
        self.n += len(self.knowledge.user_dialog_actions)
        self.n += len(self.knowledge.goal_actions) + 1
        self.n *= len(self.knowledge.summary_system_actions)
        
        # Initialize params
        mu = 0
        sigma = self.knowledge.ktdq_init_theta_std_dev
        self.theta = np.matrix(np.random.normal(mu, sigma, self.n)).T
        self.P = self.knowledge.ktdq_lambda * np.matrix(np.eye(self.n))
        self.eta = self.knowledge.ktdq_eta
        self.P_n = self.knowledge.ktdq_P_n
        self.kappa = self.knowledge.ktdq_kappa
        self.gamma = self.knowledge.gamma 
        
        self.cleaning_epsilon = self.knowledge.ktdq_cleaning_epsilon
        self.alpha = self.knowledge.ktdq_alpha
        self.beta = self.knowledge.ktdq_beta

    # Convert the summary state feature vector b to the features used by
    # the algorithm and get feature vector for the case when action is a
    def get_feature_vector(self, b, a) :
        orig_feature_vector = b.get_feature_vector()
        new_state_feature_vector = list()
        new_state_feature_vector.append(1)
        
        s1 = orig_feature_vector[0]     # Probability of top hypothesis
        s2 = orig_feature_vector[1]     # Probability of second hypothesis
        for (c1, c2) in self.knowledge.ktdq_rbf_centres :
            dist_to_rbf_centre = (s1 - c1) ** 2 + (s2 - c2) ** 2
            feature_value = math.exp(-dist_to_rbf_centre / 2 * (self.knowledge.ktdq_rbf_sigma ** 2))
            # exp(-((s1 - c1)^2 + (s2 - c2)^2) / 2 * sigma^2)
            new_state_feature_vector.append(feature_value)
        
        # No of goals allowed by the top partition     
        s3 = orig_feature_vector[2]
        # Delta function for each possible value
        for i in range(1, len(self.knowledge.goal_actions) + 1) :
            new_state_feature_vector.append(int(i == s3))

        # No of params in the top partition required by its action that are uncertain    
        s4 = orig_feature_vector[3]
        # Delta function for each possible value
        for i in range(0, len(self.knowledge.goal_params) + 1) :
            new_state_feature_vector.append(int(i == s4))
        
        # Number of dialog turns used so far    
        s5 = orig_feature_vector[4]        
        # 0-1 : Is dialogue too long?
        new_state_feature_vector.append(int(s5 > self.knowledge.ktdq_long_dialogue_thresh))
        
        # Do the top and second hypothesis use the same partition: yes/no  
        s6 = orig_feature_vector[5]
        # 0-1 : Do the top and second hypothesis use the same partition?
        new_state_feature_vector.append(int(s6 == 'yes'))

        # Type of last user utterance - inform_full/inform_param/affirm/deny        
        s7 = orig_feature_vector[6]
        # Delta function for each possible value
        for value in self.knowledge.user_dialog_actions :
            new_state_feature_vector.append(int(s7 == value))    
            
        # Goal in top hypothesis partition or 'None' if this is not unique
        s8 = orig_feature_vector[7]
        # Delta function for each possible value
        for value in self.knowledge.goal_actions + [None] :
            new_state_feature_vector.append(int(s8 == value))
            
        # For each feature in the state feature vectors, create 4 
        # features by multiplying with delta function of each action
        new_feature_vector = list()
        for value in new_state_feature_vector :
            for action in self.knowledge.summary_system_actions :
                new_feature_vector.append(value * int(action == a))        
                
        return np.matrix(new_feature_vector).T
    
    # Calculate Q(b, a) using current parameters
    def calc_q(self, b, a) :
        f = self.get_feature_vector(b, a)
        print 'f = ', f.shape
        print 'self.theta = ', self.theta.shape
        return (f.T * self.theta).item(0, 0)
                
    # Pick a random action that is valid for the current state 
    def get_random_action(self, current_state) :
        candidate_actions = self.get_candidate_actions(current_state)
        num_actions = len(candidate_actions)
        sample = int(np.random.uniform(0, num_actions))
        return candidate_actions[sample]
    
    # Return the set of actions allowed for the current state. These
    # are mostly hand-coded rules
    def get_candidate_actions(self, current_state) :
        candidate_actions = copy.deepcopy(self.knowledge.summary_system_actions)
        
        # Allow 'take-action' only if the top hypothesis has a unique 
        # executable action
        if current_state is None or current_state.top_hypothesis is None :
            candidate_actions.remove('take_action')
            return candidate_actions
        elif current_state.top_hypothesis[0] is None or current_state.top_hypothesis[0].possible_goals is None :
            candidate_actions.remove('take_action')
            return candidate_actions
        elif len(current_state.top_hypothesis[0].possible_goals) != 1 :
            candidate_actions.remove('take_action')
            return candidate_actions
        else :
            # Do not allow 'request-missing-param' for speak actions
            unique_goal = current_state.top_hypothesis[0].possible_goals[0]
            if unique_goal == 'speak_t' or unique_goal == 'speak_e' :
                if 'request_missing_param' in candidate_actions :
                    candidate_actions.remove('request_missing_param')
                    
            # Do not allow 'take-action' if some required param is uncertain
            for param_name in self.knowledge.param_order[unique_goal] :
                if param_name not in current_state.top_hypothesis[0].possible_param_values :
                    if 'take_action' in candidate_actions :
                        candidate_actions.remove('take_action')
                elif len(current_state.top_hypothesis[0].possible_param_values[param_name]) != 1 :
                    if 'take_action' in candidate_actions :
                        candidate_actions.remove('take_action')
            return candidate_actions
    
    # Defines the current policy
    def pi(self, b) :
        r = np.random.uniform()
        if r < self.knowledge.ktdq_epsilon :
            # Pick random action
            return self.get_random_action(b)
        else :
            candidate_actions = self.get_candidate_actions(b)
            max_q = -sys.maxint
            best_action = None
            for a in candidate_actions :
                q = self.calc_q(b, a)
                if q >= max_q :
                    max_q = q
                    best_action = a
            # Greedy action            
            return best_action        
    
    def get_initial_action(self, initial_state) :
        a = 'repeat_goal'
        return (a, self.get_system_action_requirements(a, initial_state))
    
    def get_next_action(self, reward, current_state) :
        a = self.pi(current_state)
        return (a, self.get_system_action_requirements(a, current_state))
    
    def update_final_reward(self, reward) :
        pass
    
    def check_covariance_matrix(self, m) :
        n = m.shape[0]
        #for i in range(n) :
            #if m[i, i] < 0 :
                #print 'Diagonal element is < 0'
                #print 'C = ', self.C
                #sys.exit(1)
        for i in range(n) :
            for j in range(n) :
                if m[i, j] != m[j, i] :
                    print 'Matrix not symmetric'
                    sys.exit(1)
        (eig_vals, eig_vectors) = np.linalg.eig(m)
        for eig_val in eig_vals :
            if type(eig_val) in [np.complex_, np.complex64, np.complex128] :
                print 'Complex eigen value'
                #print 'C = ', self.C
                sys.exit(1)
            elif eig_val < -0.0001 :
                print 'Negative eigen value'
                sys.exit(1)
   
    def compute_sigma_points(self) :
        theta = list()
        w = list()
        
        ktdq_lambda = (self.alpha ** 2) * (self.n + self.kappa) - self.n
        
        matrix_to_decompose = (self.n + ktdq_lambda) * self.P
        #print 'matrix_to_decompose = ', matrix_to_decompose
        #self.check_covariance_matrix(matrix_to_decompose)
        cholesky_decomposition = np.linalg.cholesky(matrix_to_decompose)
        
        for j in range(0, 2 * self.n + 1) :
            if j == 0 :
                theta_j = self.theta
                #w_j = self.kappa / (self.n + self.kappa)
                #w_j = (w_j / (self.alpha * self.alpha)) + (1.0 / (self.alpha * self.alpha)) - 1
                w_j = ktdq_lambda / self.n + ktdq_lambda
            elif j <= self.n :
                # Add j^th column of cholesky decomposition
                theta_j = self.theta + cholesky_decomposition[:, j-1]
                #w_j = 1.0 / (self.n + self.kappa)
                #w_j = (w_j / (self.alpha * self.alpha))
                w_j = 1.0 / 2 * (self.n + ktdq_lambda)
            else :
                # Subtract (j-n)^th column of cholesky decomposition
                theta_j = self.theta - cholesky_decomposition[:, j-self.n-1]
                #w_j = 1.0 / 2 * (self.n + self.kappa)
                #w_j = (w_j / (self.alpha * self.alpha))
                w_j = 1.0 / 2 * (self.n + ktdq_lambda)
                
            #theta_j = self.theta + self.alpha * (theta_j - self.theta)
                
            theta.append(theta_j)
            w.append(w_j)
                
            #a = np.random.multivariate_normal(np.array(self.theta.flatten()).flatten(), self.P)
            #theta.append(np.matrix(np.reshape(a, (a.size, 1))))
            #w.append(1)    
        return (theta, w)
    
    # Train on a single example (b, a, b', r)
    def train(self, b, a, b_prime, r) :
        #print 'theta = ', self.theta
        #print 'P = ', self.P
        #raw_input()
        
        # Prediction step
        P_v = self.eta * self.P     
        self.P = self.P + P_v       
        
        # Calculate sigma points - thetas, weights - w for unscented transform
        # Calculate statistics at the same time
        r_hat = 0
        P_theta_r = self.get_zero_vector(self.n)
        P_r = self.P_n
        (theta, w) = self.compute_sigma_points()
        for j in range(0, 2 * self.n + 1) :
            f = self.get_feature_vector(b, a)
            r_j = (f.T * theta[j]).item(0, 0)
            
            if b_prime is not None :
                # If it is a non-terminal state account for future rewards
                max_q = -sys.maxint
                for a_prime in self.get_candidate_actions(b_prime) :
                    f = self.get_feature_vector(b_prime, a_prime)            
                    q = (f.T * theta[j]).item(0, 0)
                    if q > max_q :
                        max_q = q
                f = self.get_feature_vector(b, a)        
                r_j -= self.gamma * max_q
            
            r_hat += r_j * w[j]

            if j == 0 :
                P_r += (w[j] + 1 - (self.alpha ** 2) + self.beta) * (r_j - r_hat) * (r_j - r_hat)
                P_theta_r += ((w[j] + 1 - (self.alpha ** 2) + self.beta) * (r_j - r_hat)) * (theta[j] - self.theta)
            else :
                P_r += w[j] * (r_j - r_hat) * (r_j - r_hat)
                P_theta_r += (w[j] * (r_j - r_hat)) * (theta[j] - self.theta)
        
        #print 'P_r = ', P_r
        #raw_input()
        #print 'P_theta_r = ', P_theta_r
        #raw_input()
        # Correction step    
        K = (1.0 / P_r) * P_theta_r
        #print 'K = ', K
        #raw_input()
        self.theta += K * (r - r_hat)
        self.P -= P_r * (K * K.T)
        #print 'theta = ', self.theta
        #raw_input()
        #print 'P = ', self.P
        #raw_input()
        
        # Clean up to keep P positive definite
        #self.P = 0.5 * self.P + 0.5 * self.P.T
        #self.P = self.P + self.cleaning_epsilon * np.eye(self.n)
        
        self.save_vars()

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
            if state.top_hypothesis is None or state.top_hypothesis[0] is None :
                # No hypotheses to verify. Confirm something random
                goal_idx = int(np.random.uniform(0, len(state.knowledge.goal_actions)))
                return SystemAction(action_type, state.knowledge.goal_actions[goal_idx])
            
            goal = None
            if len(state.top_hypothesis[0].possible_goals) > 0 : 
                goal_idx = int(np.random.uniform(0, len(state.top_hypothesis[0].possible_goals)))    
                goal = state.top_hypothesis[0].possible_goals[goal_idx]
            else :
                # Top hypothesis has no goals, verify any random goal
                goal_idx = int(np.random.uniform(0, len(self.knowledge.possible_goals)))    
                goal = state.top_hypothesis[0].possible_goals[goal_idx]

            system_action = SystemAction(action_type, goal)
            param_order = state.knowledge.param_order[goal]
            params = dict()
            partition_params = state.top_hypothesis[0].possible_param_values
            if partition_params is None :
                return system_action
            for param_name in param_order :
                if param_name in partition_params and len(partition_params[param_name]) == 1 :
                    # Only confirm params if the hypothesis thinks there is only one 
                    # possible value for it
                    params[param_name] = partition_params[param_name][0]
                #else :
                    #print 'Uncertain about ', param_name, ' : ', partition_params[param_name]
            system_action.referring_params = params
            return system_action
            
        elif action_type == 'request_missing_param' :
            if state.top_hypothesis is None or state.top_hypothesis[0] is None :
                # No hypotheses. Anything can be asked
                goal_idx = int(np.random.uniform(0, len(state.knowledge.goal_actions)))
                goal = state.knowledge.goal_actions[goal_idx]
                system_action = SystemAction(action_type, goal)
                param_idx = int(np.random.uniform(0, len(state.knowledge.param_order[goal])))
                system_action.extra_data = [state.knowledge.param_order[goal][param_idx]]
                return system_action
                    
            goal = state.top_hypothesis[0].possible_goals[0]
            system_action = SystemAction(action_type, goal)
            param_order = state.knowledge.param_order[goal]
            params = dict()
            uncertain_params = list()
            partition_params = state.top_hypothesis[0].possible_param_values
            if partition_params is None :
                return system_action
            for param_name in param_order :
                if param_name not in partition_params or len(partition_params[param_name]) != 1 :
                    uncertain_params.append(param_name)
                else :
                    params[param_name] = partition_params[param_name][0]
            system_action.referring_params = params
            if len(uncertain_params) > 0 :
                # If the top partition is uncertain about a param, confirm it
                param_idx = int(np.random.uniform(0, len(uncertain_params)))
                system_action.extra_data = [uncertain_params[param_idx]]
                return system_action
            else :
                # The top hypothesis partition doesn't have uncertain 
                # params but it is possible it is not of high enough 
                # confidence
                
                # If there is no second hypothesis, just confirm any value
                # This param si chosen at random so that you don't get 
                # stuck in a loop here
                if state.second_hypothesis is None or state.second_hypothesis[0].possible_param_values is None :
                    param_idx = int(np.random.uniform(0, len(param_order)))
                    system_action.extra_data = [param_order[param_idx]]
                    return system_action
                
                # A good heuristic is to see in what params the first 
                # and second hypotheses differ. Any one of these is 
                # likely to help. 
                second_params = state.second_hypothesis[0].possible_param_values
                for param_name in param_order :
                    top_param_value = partition_params[param_name][0]
                    if param_name not in second_params or top_param_value not in second_params[param_name] or len(second_params[param_name]) == 1 :
                        # This is not a useful param to compare
                        pass
                    else :
                        system_action.extra_data = [param_name]
                        return system_action
                
                # If you reached here, this is probably an inappropriate 
                # action so just verify a random param. 
                param_idx = int(np.random.uniform(0, len(param_order)))
                system_action.extra_data = [param_order[param_idx]]
                return system_action
                
            return system_action
                    
    def save_vars(self) :
        save_model(self.theta, 'theta')
        save_model(self.P, 'P')

    def print_vars(self) :
        print '------------------------------------------------'
        print 'self.theta = ', self.theta
        print 'self.P = ', self.P
        print '------------------------------------------------'
       
    def get_action_from_hand_coded_policy(self, state) :
        if state.num_dialog_turns > 10 :
            return 'take_action'
        feature_vector = state.get_feature_vector()
        num_goals = feature_vector[2]
        num_uncertain_params = feature_vector[3]
        last_utterance_type = feature_vector[6]
        if num_goals == 1 :
            if num_uncertain_params == 0 :
                if state.top_hypothesis_prob < 0.3 :
                    return 'request_missing_param'
                if last_utterance_type == 'affirm' or last_utterance_type == 'deny' :
                    if state.top_hypothesis_prob < 0.75 :
                        return 'repeat_goal'      
                    else :
                        return 'take_action'
                else :
                    if state.top_hypothesis_prob < 0.99 :
                        return 'confirm_action'
                    else :
                        return 'take_action'
            else :
                return 'request_missing_param'
        else :
            return 'repeat_goal'      
        
                   
        
    
        
