__author__ = 'aishwarya'

from Utils import *

class Utterance:

    def __init__(self, action_type, referring_goal=None, referring_params=None, parse_prob=0.0):
        # This gives the type of dialog action
        # Possible types - inform, affirm, deny
        # In case of affirm and deny, the goal and params will be taken
        # from the system sction that caused the user to affirm or deny
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
        
# Simple tests to check syntax    
if __name__ == '__main__' :
    u = [0,0,0,0,0,0,0,0,0,0]
    u[0] = Utterance('affirm', 'searchRoom', {'Patient':'ray', 'Location':'l3512'})
    u[1] = Utterance('affirm', 'searchRoom', {'Patient':'ray', 'Location':'l3512'})
    u[2] = Utterance('deny', 'searchRoom', {'Patient':'ray', 'Location':'l3512'})
    u[3] = Utterance('inform', 'searchRoom', {'Patient':'ray', 'Location':'l3512'})
    u[4] = Utterance('affirm', 'SearchRoom', {'Patient':'ray'})
    u[5] = Utterance('affirm', 'searchRoom', {'Patient':'ray', 'Recipient':'peter'})
    u[6] = Utterance('affirm', 'SearchRoom', {})
    u[7] = Utterance('inform', 'SearchRoom')
    u[8] = Utterance('affirm', 'remind', {'Patient':'ray', 'Location':'l3512'})
    print str(u[0])
    for i in xrange(1,9) :
        print u[0] == u[i]
    
