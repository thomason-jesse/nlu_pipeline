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
        
    def get_matching_partitions(self, system_action, utterance) :
        matching_partitions = []
        for partition in self.partitions :
            if partition.match(system_action, utterance) :
                matching_partitions.append(partition)
        return matching_partitions
        
    def make_all_matching_partitions(self, system_action, utterance) :
        matching_partitions = []
        new_leaf_partitions = self.partitions
        required_goal = None
        if system_action.referring_goal != None :
            required_goal = system_action.referring_goal
        elif utterance.referring_goal != None :
            required_goal = utterance.referring_goal    
        required_params = dict()
        if system_action.referring_params != None :
            for param_name in system_action.referring_params :
                if param_name in required_params :
                    required_params[param_name] = required_params[param_name] + system_action.referring_params[param_name]
                else :
                    required_params[param_name] = system_action.referring_params[param_name]
        if utterance.referring_params != None :
            for param_name in utterance.referring_params :
                if param_name in required_params :
                    required_params[param_name] = required_params[param_name] + utterance.referring_params[param_name]
                else :
                    required_params[param_name] = utterance.referring_params[param_name]
        
        # This is incomplete!!! Now iterate over the partitions and 
        # split any partition that is a superset of what is required
        for partition in self.partitions :
            if partition.is_superset(required_goal, required_params) :
                # This split is non-trivial. Perhaps it is better to 
                # do multiple iterations of splitting - first for goal 
                # then for each param
                # WARNING: INCOMPLETE!!!!!
                
        
# Simple tests to check for syntax errors
if __name__ == '__main__' :
    knowledge = Knowledge()
    b = HisBeliefState(knowledge)
