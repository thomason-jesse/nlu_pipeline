# WARNING: This is not really being used and is incomplete. I wanted to implement this 
# but temporarily ditched it in favour of HIS because I actually understand that

__author__ = 'aishwarya'

class BudsBeliefState:

    def __init__(self, knowledge):

        self.knowledge = knowledge
        self.num_user_turns = 0

        # Number of possible actions the goal could be referring to
        self.num_goal_actions = len(goal_actions) 
        self.num_utterance_actions = self.num_goal_actions + 1 # As action is utterance is allowed to be None  

        # n - Maximum possible number of parameters an action can have
        self.max_goal_params = len(goal_params)     

        # Number of possible values each parameter can take
        self.num_goal_param_values = len(goal_params_values) 

        # Initialize all belief variables to uniform distributions

        # g_a - Belief of the goal of (final action desired by) user
        self.goal_action_belief = [1.0/self.num_goal_actions] * self.num_goal_actions

        # g_p_i - Belief of the value of each param. For now this is a list of lists 
        # 		  but hopefully can be made a matrix later for efficient computation
        self.goal_params_belief = [[1.0/self.num_goal_param_values for x in range(self.num_goal_param_values)] for x in range(num_goal_params)] 

        # u_a - Belief of things clarified in previous user utterance
        self.utterance_action_belief = [1.0/self.num_goal_actions] * self.num_goal_actions

        # u_p_i - Belief of the value of each param clarified in previous user utterance. 
        # 		  For now this is a list of lists but hopefully can be made a matrix 
        #		  later for efficient computation
        self.utterance_params_belief = [[1.0/self.num_goal_param_values for x in range(self.num_goal_param_values)] for x in range(num_goal_params)] 

        # Scores of best N parses (it doesn't matter what N is but having more values is better)
        self.best_n_parse_scores = []

        # Create a list of all factor node names. This is a more cumbersome implementation
        # but it will be easier to extend if we add more information to the state        
        self.cond_dep_factors_names = ['f_g_a', 'f_u_a', 'f_o']
        for i in xrange(0, self.max_goal_params) :
            self.cond_dep_factors_names = self.cond_dep_factors_names + ['f_v_g_' + str(i), 'f_g_p_' + str(i), 'f_v_u_' + str(i), 'f_u_p_' + str(i)]

    # Calculates the conditional probability f_g_a(g_a' / g_a, m)
    # Assumes goal actions passed are atomic
    def calc_goal_action_factor(self, new_goal_action, old_goal_action, system_action) :
        if new_goal_action == old_goal_action :
            return (1.0 - self.knowledge.goal_change_prob)
        else :
            return self.knowledge.goal_change_prob / self.num_goal_actions

    # Calculates the conditional probability f_g_p_i(g_p_i' / g_p_i, m)
    # Assumes param values are atomic
    def calc_goal_param_factor(self, i, new_goal_param, old_goal_param, system_action) :
        if new_goal_param == old_goal_param :
            return (1.0 - self.knowledge.goal_change_prob)
        else :
            return self.knowledge.goal_change_prob / self.num_goal_param_values

    # Calculates the conditional probability f_v_g_i(v_g_i' / g_a')
    def calc_goal_param_relevance_factor(self, i, relevance_value, new_goal_action) :
        if relevance_value == self.knowledge[new_goal_action][self.knowledge.goal_params[i]] :
            return 1.0
        else :
            return 0.0

    # Calculates the conditional probability f_v_u_i(v_u_i' / u_a')
    def calc_utterance_param_relevance_factor(self, i, relevance_value, utterance_action) :
        if relevance_value == self.knowledge[utterance_action][self.knowledge.goal_params[i]] :
            return 1.0
        else :
            return 0.0

    # Calculates the conditional probability f_u_a(u_a' / g_a', m)
    def calc_utterance_action_factor(self, utterance_action, goal_action, system_action) :
        if system_action.name == 'repeat_goal' :
            if utterance_action == goal_action :
                return 1.0
            else :
                return 0.0
        else :
            if utterance_action == None : 
                return 1.0
            else :
                return 0.0

    # Calculates the conditional probability f_u_a(u_a' / g_a', m)
    def calc_utterance_action_factor(self, utterance_action, goal_action, system_action) :
        if system_action.name == 'repeat_goal' :
            if utterance_action == goal_action :
                return 1.0
            else :
                return 0.0
        else :
            if utterance_action == None : 
                return 1.0
            else :
                return 0.0

    # Calculates the conditional probability f_u_p_i(u_p_i' / v_u_i', m, g_p_i')
    def calc_utterance_param_factor(self, i, utterance_param_value, relevance, system_action, goal_param_value) :
        if relevance == 0 :
            if utterance_param_value == None :
                return 1.0
            else :
                return 0.0
        
        if system_action.name == 'repeat_goal' :
            if utterance_param_value == goal_param_value :
                return 1.0
            else :
                return 0.0
        elif system_action.name == 'request_missing_param' and knowledge.goal_params[i] in system_action.referring_params :
            if utterance_param_value == goal_param_value :
                return 1.0
            else :
                return 0.0            
        else :
            if utterance_action == None : 
                return 1.0
            else :
                return 0.0

    # Calculates the conditional probability f_o(o' / u_a', u_p_i')
    # Requires utterance_params to be a dict
    def calc_observation_factor(self, observation, utterance_action, utterance_params) :
        


