__author__ = 'aishwarya'

from Partition import Partition
from Knowledge import Knowledge     # Only needed for testing
from SystemAction import SystemAction
from Utterance import Utterance

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
        possible_param_values['patient'] = knowledge.goal_params_values
        possible_param_values['location'] = knowledge.goal_params_values
        possible_param_values['recipient'] = knowledge.goal_params_values
        complete_partition = Partition(possible_actions, possible_param_values, 1.0)
        self.partitions = [complete_partition]	
        
    def get_matching_partitions(self, system_action, utterance) :
        matching_partitions = []
        for partition in self.partitions :
            if partition.match(system_action, utterance) :
                matching_partitions.append(partition)
        return matching_partitions
    
    # Warning: This function only creates matching partitions in the sense
    # that they match in goal and params. In the case of a confirm system
    # action, it does not check whether the user said yes or no    
    def make_all_matching_partitions(self, system_action, utterance) :
        if system_action.name == 'confirm_action' :
            if utterance.extra_data is None or knowledge.no in utterance.extra_data :
                return # If the user did not confirm, then no need to split any partition
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
                    required_params[param_name] = [system_action.referring_params[param_name]]
        if utterance.referring_params != None :
            for param_name in utterance.referring_params :
                if param_name in required_params :
                    required_params[param_name] = required_params[param_name] + utterance.referring_params[param_name]
                else :
                    required_params[param_name] = [utterance.referring_params[param_name]]

        print "required_goal = ", required_goal
        print "required_params = ", required_params
        
        # Iterate over the partitions and split any partition that is a 
        # superset of what is required
        new_partitions = list()
        for partition in self.partitions :
            if partition.is_superset(required_goal, required_params) and not partition.is_equal(required_goal, required_params) :
                # This is a partition to be split
                goal_split_partitions = list()
                if required_goal != None :  # Split based on goal
                    goal_split_partitions = partition.split_by_goal(required_goal, knowledge)
                else :
                    goal_split_partitions = [partition]

                if required_params != None :
                    param_value_pairs = list()
                    for param_name in required_params :
                        param_value_pairs = param_value_pairs + [(param_name, value) for value in required_params[param_name]]
                    print "param_value_pairs = ", param_value_pairs, '\n'
                    if len(param_value_pairs) == 0 :
                        new_partitions = new_partitions + goal_split_partitions
                        continue
                    partitions_to_split = goal_split_partitions
                    resultant_partitions = list()
                    # For each (param_name, param_value), take all the splits
                    # you ahve already made for the current partition and
                    # split each one of them acc to this pair
                    for (param_name, param_value) in param_value_pairs :
                        resultant_partitions = list()
                        for partition in partitions_to_split :
                            resultant_partitions = resultant_partitions + partition.split_by_param(param_name, param_value, knowledge)
                        partitions_to_split = resultant_partitions
                        
                    new_partitions = new_partitions + resultant_partitions
                else :
                    # There were no params to split on. So only add the 
                    # partitions obtained by splitting the goal 
                    new_partitions = new_partitions + goal_split_partitions
            else :
                # Either this partition is not a superset of the specified 
                # goal or params, or it is already an exact match of the 
                # required goal and params so don't split it
                new_partitions = new_partitions + [partition]    
        self.partitions = new_partitions
        
# Simple tests to check for syntax errors
if __name__ == '__main__' :
    knowledge = Knowledge()
    b = HisBeliefState(knowledge)
    print 1, [str(p) for p in b.partitions], '\n'
    m1 = SystemAction('confirm_action', 'searchroom', {'patient':'ray', 'location':'3512'})
    m2 = SystemAction('repeat_goal')    
    u1 = Utterance('searchroom', {'patient':'ray', 'location':'3512'})
    u2 = Utterance(None, None, [Knowledge.yes])
    u3 = Utterance(None, None, [Knowledge.no])
    b.make_all_matching_partitions(m1, u2)
    for p in b.partitions :
        print str(p), '\n'
