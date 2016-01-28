__author__ = 'aishwarya'

import sys, math

class SummaryState :
    discrete_features = [5,6,7]
    
    def __init__(self, hisBeliefState=None) :
        # TODO: These default values don't make sense. Think about them
        self.top_hypothesis_prob = 0.0      
        self.second_hypothesis_prob = 0.0

        if hisBeliefState is not None :            
            self.knowledge = hisBeliefState.knowledge
            self.num_dialog_turns =  hisBeliefState.num_dialog_turns
        
        self.top_hypothesis = None
        self.second_hypothesis = None
        
        if hisBeliefState is not None and hisBeliefState.hypothesis_beliefs is not None :
            inverse = [(value, key) for (key,value) in hisBeliefState.hypothesis_beliefs.items()]
            self.top_hypothesis_prob = max(inverse)[0]
            self.top_hypothesis = max(inverse)[1]
            inverse.remove(max(inverse))
            if len(inverse) > 0 :
                self.second_hypothesis_prob = max(inverse)[0]
                self.second_hypothesis = max(inverse)[1]

    # Ordered list of features
    #     - Probability of top hypothesis
    #     - Probability of second hypothesis
    #     - No of goals allowed by the top partition 
    #     - No of params in the top partition required by its action that are uncertain
    #     - Number of dialog turns used so far
    #     - Do the top and second hypothesis use the same partition: yes/no  
    #     - Type of last user utterance - inform_full/inform_param/affirm/deny
    #     - Goal in top hypothesis partition or 'None' if this is not unique
    # NOTE: PomdpGpSarsaPolicy.py DEPENDS ON THIS ORDERING. 
    def get_feature_vector(self) :
        # Probability of top hypothesis; Probability of second hypothesis
        feature = [self.top_hypothesis_prob, self.second_hypothesis_prob]
        
        # No of goals allowed by the top partition 
        num_goals_allowed = len(self.knowledge.goal_actions)
        if self.top_hypothesis is not None :
            num_goals_allowed = len(self.top_hypothesis[0].possible_goals)
        feature.append(num_goals_allowed)
        
        # No of params in the top partition required by its action that are uncertain
        num_params_uncertain = 0
        #uncertain_params = []
        if num_goals_allowed == 1 :
            goal = self.top_hypothesis[0].possible_goals[0]
            param_order = self.knowledge.param_order[goal]
            #print 'goal = ', goal, 'param_order = ',param_order
            for param_name in param_order :
                if len(self.top_hypothesis[0].possible_param_values[param_name]) != 1 :
                    num_params_uncertain += 1
                    #uncertain_params.append(param_name)
        else :
            num_params_uncertain = len(self.knowledge.goal_params)
        feature.append(num_params_uncertain)           
        #print 'uncertain_params = ', uncertain_params             
        
        # Number of dialog turns used so far
        feature.append(self.num_dialog_turns)
        
        # Do the top and second hypothesis use the same partition: Yes/No 
        if self.top_hypothesis is None :
            if self.second_hypothesis is None :
                feature.append('yes')
            else :
                feature.append('no')
        else :
            if self.second_hypothesis is None :
                feature.append('no')
            elif self.top_hypothesis[0] == self.second_hypothesis[0] :
                feature.append('yes')
            else :
                feature.append('no')
            
        # Type of last user utterance
        if self.top_hypothesis is None :
            feature.append(None)
        elif type(self.top_hypothesis[1]) == str :
            feature.append(self.top_hypothesis[1])
        else :
            feature.append(self.top_hypothesis[1].action_type)
            
            
        # Goal in top hypothesis partition or 'None' if this is not unique
        if self.top_hypothesis is None or self.top_hypothesis[0] is None:
            feature.append('None')
        elif len(self.top_hypothesis[0].possible_goals) > 1 :
            feature.append('None')
        elif len(self.top_hypothesis[0].possible_goals) == 1 :
            feature.append(self.top_hypothesis[0].possible_goals[0])
        else :
            feature.append('None')
        
        return feature
        
    # Calculate the distance between this summary state and the other
    # summary state
    #def distance_to(self, other_summary_state) :
        #distance = 0.0
        #cts_distance = 0.0
        #self_feature_vector = self.get_feature_vector()
        #other_feature_vector = other_summary_state.get_feature_vector()
        #weights = self.knowledge.summary_space_distance_weights
        #for i in xrange(0, len(self_feature_vector)) :
            #if i in SummaryState.discrete_features :
                #distance += weights[i] * (1.0 - float(self_feature_vector[i] == other_feature_vector[i]))
            #else :
                #cts_distance += weights[i] * (self_feature_vector[i] - other_feature_vector[i]) * (self_feature_vector[i] - other_feature_vector[i])
        #distance += math.sqrt(cts_distance)        
        #return distance
        
    def calc_kernel(self, other_summary_state) :
        p = self.knowledge.kernel_degree
        sigma_k = self.knowledge.kernel_std_dev
        k_disc = 0.0
        norm_cts = 0.0
        self_feature_vector = self.get_feature_vector()
        other_feature_vector = other_summary_state.get_feature_vector()
        for i in xrange(0, len(self_feature_vector)) :
            if i in self.discrete_features :
                k_disc += self.knowledge.kernel_weights[i] * float(self_feature_vector[i] == other_feature_vector[i])
            else :
                norm_cts += self.knowledge.kernel_weights[i] * (self_feature_vector[i] - other_feature_vector[i]) ** 2
        k_cts = p * p * math.exp( - norm_cts / (2 * sigma_k * sigma_k))
        return k_disc + k_cts
     
        
