__author__ = 'aishwarya'

from Partition import Partition
from Knowledge import Knowledge     # Only needed for testing

class HisBeliefState:

    def __init__(self, knowledge):

        self.knowledge = knowledge
        self.num_user_turns = 0

        # Number of possible actions the goal could be referring to
        self.num_goal_actions = len(knowledge.goal_actions) 
        self.num_utterance_actions = self.num_goal_actions + 1 # As action is utterance is allowed to be None  

        # n - Maximum possible number of parameters an action can have
        self.max_goal_params = len(knowledge.goal_params)     

        # Number of possible values each parameter can take
        self.num_goal_param_values = len(knowledge.goal_params_values) 

        # Create a partition with all possible states
        possible_actions = knowledge.goal_actions
        possible_param_values = dict()
        possible_param_values['Patient'] = knowledge.goal_params_values
        possible_param_values['Location'] = knowledge.goal_params_values
        possible_param_values['Recipient'] = knowledge.goal_params_values
        complete_partition = Partition(possible_actions, possible_param_values, 1.0)
        self.partitions = [complete_partition]

# Simple tests to check for syntax errors
if __name__ == '__main__' :
    knowledge = Knowledge()
    b = HisBeliefState(knowledge)
