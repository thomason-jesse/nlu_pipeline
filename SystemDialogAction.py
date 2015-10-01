__author__ = 'aishwarya'

class SystemDialogAction:
    # A system action has a name and a dict of params
    # The dict is of the form referring_params[param_name] = param_value
    # We need param values for the confirm_action action but not for request_missing_param
    # In the latter case, we will create a dictionary entry for the param being requested 
    # but with value None

    def __init__(self, name=None, referring_goal=None, referring_params=None):
        # Name of the dialog action. Possible names can be found in Knowledge.py
        self.name = name        
        
        # What goal action is being confirmed. Relevant mainly for 'confirm_goal'
        self.referring_goal = referring_goal   

        # What params are being confirmed. Relevant mainly for 'confirm_goal'
        self.referring_params = referring_params 

    def __str__(self):
        return self.name + ':' + str(self.referring_goal) + '(' + ','.join([str(k) + ':' + str(v) for (k, v) in self.referring_params.items()]) + ')'

    # assumes values of referring_params are atomic
    def __eq__(self, other) :
        if self.name != other.name :
            return False
        if self.referring_goal != other.referring_goal :
            return False
        if type(other.referring_params != dict) :
            return False
        if len(self.referring_params.keys()) != len(other.referring_params.keys()) :
            return False
        for key in self.referring_params :
            if key not in other.referring_params :
                return False
            elif other.referring_params[key] != self.referring_params[key]
                return False 
        return True
