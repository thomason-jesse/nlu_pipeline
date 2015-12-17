__author__ = 'aishwarya'

# TODO: Automate filling as many of these as possible from knowledge base

class Knowledge:
    yes = 'yes'
    no = 'no'
    unk = '-UNK-'

    def __init__(self):
        self.goal_actions = ['searchroom', 'speak_t', 'speak_e', 'remind', 'askperson', 'bring', 'at']
        self.goal_params = ['patient', 'recipient', 'location']

        # This is kept as a single vector common to all values so that hopefully 
        # some operaitons involving them can be made matrix operations and implemented
        # efficiently using numpy
        self.goal_params_values = [None, 'peter', 'ray', 'dana', 'kazunori', 'matteo', 'shiqi', 'jivko', 'stacy', 'yuqian', 'max', 'pato', 'bwi', 'bwi_m', 'l3_516', 'l3_508', 'l3_512', 'l3_510', 'l3_402', 'l3_418', 'l3_420', 'l3_432', 'l3_502', 'l3_414b', True, False]
        
        self.system_dialog_actions = ['repeat_goal', 'confirm_action', 'request_missing_param']
        self.user_dialog_actions = ['inform', 'affirm', 'deny']

        self.goal_change_prob = 0.0

        # Prob of splitting a partition by specifying a goal
        # partition_split_goal_probs[goal_value] gives the probability
        # when the goal is goal_value
        self.partition_split_goal_probs = dict()
        
        # Prob of splitting a partition by specifying a param
        # partition_split_param_probs[param_name][param_value] gives the
        # probability of splitting by specifying value param_value for 
        # the param param_name
        self.partition_split_param_probs = dict()

        self.set_partition_split_probs()
        
        # Prob that the user says something other than what the system 
        # expects given its previous action
        self.user_wrong_action_prob = 0.01
        
        # action_type_probs[system_action_type][user_action_type] gives 
        # the prob that the utterance has type user_action_type given 
        # that the system action was of type system_action_type
        self.action_type_probs = dict()
        
        self.set_action_type_probs()
        
        # Probability that the obs is due to an utterance not in the 
        # N-best list
        self.obs_by_non_n_best_prob = 0.01
        
        # Probability that an utterance not in the N-best list matches 
        # the partition and system_action - This can probably be 
        # calculated exactly but it will be hard to do so.
        self.non_n_best_match_prob = 0.01
        
        # The system requires arguments to be passed to ASP in specific
        # orders for each action. This is not an obviously generalizable
        # order so it is hard-coded here. The following is the intended
        # interpretation of the parameters
        #   searchroom - patient (who/what to search for), location (where to search)
        #   speak_t - patient (what to say)
        #   speak_e - patient (what to say)
        #   remind - patient (event you are reminding someone of), location (where is the 
        #            event), recipient (the person you are reminding)
        #   askperson - patient (the person you ask), recipient (the person you ask about)
        #   bring - patient (what to bring), recipient (whom to bring it to)
        #   walk - location (where to go)
        self.param_order = dict()
        self.param_order['searchroom'] = ['patient', 'location']
        self.param_order['speak_t'] = ['patient']
        self.param_order['speak_e'] = ['patient']
        self.param_order['remind'] = ['recipient', 'patient', 'location']
        self.param_order['askperson'] = ['patient', 'recipient']
        self.param_order['bring'] = ['patient', 'recipient']
        self.param_order['at'] = ['location']

        self.param_relevance = dict()
        self.set_param_relevance()

        # Component-wise weights for distance metric in summary space
        # When calculating the distance, a sum of the weighted L2 norm
        # of the continuous components and weighted misclassification
        # distance of discrete components is used. If the weight vector
        # is smaller than the number of features, remaining weights will 
        # be taken as 0
        self.summary_space_distance_weights = [1.0, 1.0, 0.1, 0.1, 0.1, 0.1, 0.1]

        # Weight for summary action agreement in GP-SARSA kernel
        # State feature weights in this kernel are the summary space
        # distance weights
        self.summary_action_distance_weight = 0.1
        
        # Distance range in which summary belief points are mapped to
        # the same grid point
        self.summary_space_grid_threshold = 0.2
        
        self.summary_system_actions = self.system_dialog_actions + ['take_action']
        
        # Sparsification param nu for GP-SARSA
        self.sparsification_param = 1
        
        # Hyperparameters for polynomial kernel - values set from the paper
        self.kernel_std_dev = 5     # sigma_k in the paper
        self.kernel_degree = 4      # p in the paper
        #self.kernel_weights = [1, 1, 0.1, 0.1, 0.1, 0.1, 0.1]
        self.kernel_weights = [1, 1, 1, 1, 1, 1, 1]
        
        # Parameters for the RL problem and GP-SARSA
        self.gamma = 0.1
        self.gp_sarsa_std_dev = 5
        
        # Rewards
        self.correct_action_reward = 20
        self.wrong_action_reward = -20
        self.per_turn_reward = -1

    # This gives a 0-1 value for whether a param is relevant for an action. 
    # Values are taken from param_order
    def set_param_relevance(self) :
        self.param_relevance = dict()
        for action in self.goal_actions :
            self.param_relevance[action] = dict()
            for param in self.goal_params :
                if action in self.param_order and param in self.param_order[action] :
                    self.param_relevance[action][param] = 1
                else :
                    self.param_relevance[action][param] = 0
            
    def set_partition_split_probs(self) :
        for goal in self.goal_actions :
            self.partition_split_goal_probs[goal] = 1.0 / len(self.goal_actions)
        for param_name in self.goal_params : 
            self.partition_split_param_probs[param_name] = dict()
            for param_value in self.goal_params_values :
                self.partition_split_param_probs[param_name][param_value] = 1.0 / len(self.goal_params_values)

    def set_action_type_probs(self) :
        expected_actions = dict()
        expected_actions['repeat_goal'] = ['inform']
        expected_actions['request_missing_param'] = ['inform']
        expected_actions['confirm_action'] = ['affirm', 'deny']
        for system_dialog_action in self.system_dialog_actions :
            if system_dialog_action not in self.action_type_probs :
                self.action_type_probs[system_dialog_action] = dict()
            for user_dialog_action in self.user_dialog_actions :
                if expected_actions[system_dialog_action] == None :
                    # The system doesn't know what user action to expect
                    # So all user actions are equally probable
                    self.action_type_probs[system_dialog_action][user_dialog_action] = 1.0 / len(self.user_dialog_actions)
                else :
                    if user_dialog_action in expected_actions[system_dialog_action] :
                        # This is an expected action. Divide the probability 
                        # that the user took an expected action equally 
                        # among all expected actions
                        self.action_type_probs[system_dialog_action][user_dialog_action] = (1.0 - self.user_wrong_action_prob) / len(expected_actions[system_dialog_action])
                    else :
                        # This is an unexpected action. Divide the probability
                        # that the user took an unexpected action equally
                        # among all unexpected actions
                        self.action_type_probs[system_dialog_action][user_dialog_action] = self.user_wrong_action_prob / (len(self.user_dialog_actions) - len(expected_actions[system_dialog_action]))
                        

    
