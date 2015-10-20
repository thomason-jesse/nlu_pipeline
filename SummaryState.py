__author__ = 'aishwarya'

import sys

class SummaryState :
    
    def __init__(self, hisBeliefState) :
        # TODO: These default values don't make sense. Think about them
        self.top_hypothesis_prob = 0.0      
        self.second_hypothesis_prob = 0.0
        
        self.knowledge = hisBeliefState.knowledge
        self.num_dialog_turns =  hisBeliefState.num_dialog_turns
        
        self.top_hypothesis = None
        self.second_hypothesis = None
        
        if hisBeliefState is not None and hisBeliefState.hypothesis_beliefs is not None :
            inverse = [(value, key) for (key,value) in hisBeliefState.hypothesis_beliefs.items()]
            self.top_hypothesis_prob = max(inverse)[0]
            self.top_hypothesis = max(inverse)[1]
            inverse.remove(max(inverse))
            self.second_hypothesis_prob = max(inverse)[0]
            self.second_hypothesis = max(inverse)[1]

    # Ordered list of features
    #     - Probability of top hypothesis
    #     - Probability of second hypothesis
    #     - No of goals allowed by the top partition 
    #     - No of params in the top partition required by its action that are uncertain
    #     - Number of dialog turns used so far
    #     - Do the top and second hypothesis use the same partition: yes/no  
    #     - Type of last user utterance - inform/affirm/deny
    def get_feature_vector(self) :
        # Probability of top hypothesis; Probability of second hypothesis
        feature = [self.top_hypothesis_prob, self.second_hypothesis_prob]
        
        # No of goals allowed by the top partition 
        num_goals_allowed = len(self.top_hypothesis[0].possible_goals)
        feature.append(num_goals_allowed)
        
        # No of params in the top partition required by its action that are uncertain
        num_params_uncertain = 0
        if num_goals_allowed == 1 :
            goal = self.top_hypothesis[0].possible_goals[0]
            param_order = self.knowledge.param_order[goal]
            for param_name in param_order :
                if len(self.top_hypothesis[0].possible_param_values) != 1 :
                    num_params_uncertain += 1
        else :
            num_params_uncertain = sys.maxint
        feature.append(num_params_uncertain)                        
        
        # Number of dialog turns used so far
        feature.append(self.num_dialog_turns)
        
        # Do the top and second hypothesis use the same partition: Yes/No  
        if self.top_hypothesis[0] == self.second_hypothesis[0] :
            feature.append('yes')
        else :
            feature.append('no')
            
        # Type of last user utterance
        feature.append(self.top_hypothesis[1].action_type)
        
        return feature
