__author__ = 'aishwarya'

from Utils import *

class Utterance:

    def __init__(self, action_type, referring_goal=None, referring_params=None, parse_prob=0.0):
        # This gives the type of dialog action
        # Possible types - inform, affirm, deny
        # In case of affirm and deny, the goal and params will be None
        self.action_type = action_type
        
        # Best guess of goal action indicated
        self.referring_goal = referring_goal   

        # Best guess of params indicated
        self.referring_params = referring_params 
        
        # Probability of the utterance given the most recent observation
        self.parse_prob = parse_prob

    def __str__(self):
        str_form = 'UserDialogAction: '+ str(self.action_type) + '\n'
        str_form = str_form + '\tGoal: ' + str(self.referring_goal) + '\n'
        if self.referring_params is not None :
            str_form = str_form + '\tParams: ' + ','.join([str(k) + ':' + str(v) for (k, v) in self.referring_params.items()]) + '\n'
        else:
            str_form = str_form + '\tParams:None\n'
        str_form = str_form + '\tProb:' + str(self.parse_prob) + '\n'
        return str_form        

    # assumes elements of referring_params list are atomic
    def __eq__(self, other) :
        if self.action_type != other.action_type :
            return False
        if self.referring_goal != other.referring_goal :
            return False
        if not checkDicts(self.referring_params, other.referring_params) :
            return False
        if self.parse_prob != other.parse_prob :
            return False
        return True
        
    def match(self, partition, system_action) :
        if self.action_type == 'deny' :
            return True # Deny matches anything
        elif self.action_type == 'affirm' :
            # Goal and param values will be in the system action. These 
            # should be compatible with the partition
            if system_action.referring_goal != None and system_action.referring_goal not in partition.possible_goals :
                return False
            if system_action.referring_params != None :
                for param_name in system_action.referring_params :
                    if param_name not in partition.possible_param_values or system_action.referring_params[param_name] not in partition.possible_param_values[param_name] :
                        return False
            return True
            
        # Control comes here if the utterance is information providing (not affirm/deny)
        if self.referring_goal != None :
            if system_action.referring_goal != None and self.referring_goal != system_action.referring_goal :
                # Normally utterance and system_action will not both have
                # a goal but if they do, they should match
                return False
            if partition.possible_goals == None or self.referring_goal not in partition.possible_goals :
                # The partition does not allow the desired goal
                return False
        if self.referring_params != None :
            for param_name in self.referring_params :
                if system_action.referring_params != None :
                    if param_name not in system_action.referring_params or system_action.referring_params[param_name] != self.referring_params[param_name] :
                        # Any param present in both the system action and the 
                        # utterance must match. Note that a param need not and 
                        # generally doesn't exist in both
                        return False
                if param_name not in partition.possible_param_values or self.referring_params[param_name] not in partition.possible_param_values[param_name] :
                    # Typically all partitions have some set of values 
                    # for all param names. The partition must allow 
                    # the values specified in the utterance
                    return False
        return True
        
