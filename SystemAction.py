__author__ = 'aishwarya'

from Utils import *

class SystemAction:
    # A system action has a name and a dict of params
    # The dict is of the form referring_params[param_name] = param_value
    # We need param values for the confirm_action action but not for request_missing_param
    # In the latter case, we will create a dictionary entry for the param being requested 
    # but with value None

    def __init__(self, action_type, referring_goal=None, referring_params=None):
        # Like repeat_goal, request_missing_param
        self.action_type = action_type
    
        # What goal action is being confirmed. Relevant mainly for 'confirm_goal'
        self.referring_goal = referring_goal   

        # What params are being confirmed. Relevant mainly for 'confirm_goal'
        self.referring_params = referring_params 

    def __str__(self):
        str_form = 'SystemAction: ' + str(self.action_type) + '\n' 
        str_form = str_form + '\tGoal: ' + str(self.referring_goal) + '\n'
        if self.referring_params is not None :
            str_form = str_form + '\tParams:' + ','.join([str(k) + ':' + str(v) for (k, v) in self.referring_params.items()]) + '\n'
        else:
            str_form = str_form + '\tParams:None\n'
        return str_form

    # assumes values of referring_params are atomic
    def __eq__(self, other) :
        if self.action_type != other.action_type :
            return False
        if self.referring_goal != other.referring_goal :
            return False
        elif not checkDicts(self.referring_params, other.referring_params) :
            return False
        return True
    

    
