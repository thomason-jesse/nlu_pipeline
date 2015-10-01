__author__ = 'aishwarya'

class UserDialogAction:

    def __init__(self, extra_data=None, referring_goal=None, referring_params=None):
        self.standard_extra_data = ['yes', 'no', '-UNK-']

        # This encodes any knowledge other than the goal action and params
        # Typically for yes and no
        self.extra_data = extra_data        
        
        # Best guess of goal action indicated
        self.referring_goal = referring_goal   

        # Best guess of params indicated
        self.referring_params = referring_params 

    def __str__(self):
        return 'Observation: '+'('+','.join([str(self.referring_goal)] + [str(p) for p in self.referring_params] + [str(d) for d in self.extra_data])+')'

    # assumes elements of referring_params list and extra_data list are atomic
    def __eq__(self, other) :
        if self.referring_goal != other.referring_goal :
            return False
        if len(self.referring_params) != len(other.referring_params) :
            return False
        for i in range(0, len(self.referring_params)) :
            if self.referring_params[i] != other.referring_params[i]:
                return False
        if len(self.extra_data) != len(other.extra_data) :
            return False
        for i in range(0, len(self.extra_data)) :
            if self.extra_data[i] != other.extra_data[i]:
                return False
        return True
