__author__ = 'aishwarya'

# TODO: Automate filling as many of these as possible from knowledge base

class Knowledge:
    yes = 'yes'
    no = 'no'
    unk = '-UNK-'

    def __init__(self):
        self.goal_actions = ['searchroom', 'speak_t', 'speak_e', 'remind', 'askperson']
        self.goal_params = ['patient', 'location', 'recipient']

        # This is kept as a single vector common to all values so that hopefully 
        # some operaitons involving them can be made matrix operations and implemented
        # efficiently using numpy
        self.goal_params_values = [None, 'peter', 'ray', 'dana', 'kazunori', 'matteo', 'shiqi', 'jivko', 'stacy', 'yuqian', 'max', 'pato', 'bwi', 'bwi-meeting', '3516', '3508', '3512', '3510', '3402', '3418', '3420', '3432', '3502', '3414b']
        
        self.system_dialog_actions = ['repeat_goal', 'confirm_action', 'request_missing_param']
        self.user_dialog_actions = ['inform', 'affirm', 'deny']

        self.goal_change_prob = 0.0
        
        self.param_relevance = dict()
        self.set_param_relevance()

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

    # This gives a 0-1 value for whether a param is relevant for an action. 
    # Assumes the following form of actions - 
    #   searchroom - patient (who/what to search for), location (where to search)
    #   speak_t - patient (what to say)
    #   speak_e - patient (what to say)
    #   remind - patient (event you are reminding someone of), location (where is the 
    #            event), recipient (the person you are reminding)
    #   askperson - patient (the person you ask), recipient (the person you ask about)
    def set_param_relevance(self) :
        self.param_relevance = dict()
        for action in self.goal_actions :
            self.param_relevance[action] = dict()
            for param in self.goal_params :
                self.param_relevance[action][param] = 0
        
        self.param_relevance['searchroom']['patient'] = 1
        self.param_relevance['searchroom']['location'] = 1     
        self.param_relevance['speak_t']['patient'] = 1
        self.param_relevance['speak_e']['patient'] = 1
        self.param_relevance['remind']['patient'] = 1
        self.param_relevance['remind']['location'] = 1
        self.param_relevance['remind']['recipient'] = 1
        self.param_relevance['askperson']['patient'] = 1
        self.param_relevance['askperson']['recipient'] = 1

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
                        

    
