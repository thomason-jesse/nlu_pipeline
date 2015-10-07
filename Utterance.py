__author__ = 'aishwarya'

from Utils import *

class Utterance:

    def __init__(self, referring_goal=None, referring_params=None, extra_data=None):
        # This encodes any knowledge other than the goal action and params
        # Typically for yes and no
        self.extra_data = extra_data        
        
        # Best guess of goal action indicated
        self.referring_goal = referring_goal   

        # Best guess of params indicated
        self.referring_params = referring_params 

    def __str__(self):
        str_form = 'UserDialogAction: '+'(' + str(self.referring_goal) + ';'
        if self.referring_params is not None :
            str_form = str_form + ','.join([str(k) + ':' + str(v) for (k, v) in self.referring_params.items()]) + ';'
        else:
            str_form = str_form + 'None' + ';'
        if self.extra_data is not None :
            str_form = str_form + ','.join([str(d) for d in self.extra_data]) + ')'
        else:
            str_form = str_form + 'None' + ')'
        return str_form        

    # assumes elements of referring_params list and extra_data list are atomic
    def __eq__(self, other) :
        if self.referring_goal != other.referring_goal :
            return False
        if not checkDicts(self.referring_params, other.referring_params) :
            return False
        if not checkLists(self.extra_data, other.extra_data) :
            return False
        return True
        
# Simple tests to check syntax    
if __name__ == '__main__' :
    u = [0,0,0,0,0,0,0,0,0,0]
    u[0] = Utterance('SearchRoom', {'Patient':'ray', 'Location':'l3512'}, ['Yes'])
    u[1] = Utterance('SearchRoom', {'Patient':'ray', 'Location':'l3512'}, ['Yes'])
    u[2] = Utterance('SearchRoom', {'Patient':'ray', 'Location':'l3512'}, ['No'])
    u[3] = Utterance('SearchRoom', {'Patient':'ray', 'Location':'l3512'})
    u[4] = Utterance('SearchRoom', {'Patient':'ray'}, ['Yes'])
    u[5] = Utterance('SearchRoom', {'Patient':'ray', 'Recipient':'peter'}, ['Yes'])
    u[6] = Utterance('SearchRoom', {}, ['Yes'])
    u[7] = Utterance('SearchRoom')
    u[8] = Utterance('Remind', {'Patient':'ray', 'Location':'l3512'}, ['Yes'])
    print str(u[0])
    for i in xrange(1,9) :
        print u[0] == u[i]
    
