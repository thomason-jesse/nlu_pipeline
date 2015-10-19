__author__ = 'aishwarya'

class SummaryState :
    
    def __ init__(self, hisBeliefState) :
        # TODO: These default values don't make sense. Think about them
        self.top_hypothesis_prob = 1.0      
        self.second_hypothesis_prob = 0.0
        
        if hisBeliefState is not None and hisBeliefState.hypothesis_beliefs is not None :
            hypothesis_beliefs_copy = dict(hypothesis_beliefs)
            max_belief_hypothesis = max(hypothesis_beliefs)
            self.top_hypothesis_prob = hypothesis_beliefs[max_belief_hypothesis]
