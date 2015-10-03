__author__ = 'aishwarya'

from Utils import *
from Knowledge import Knowledge

class Partition:

    def __init__(self, possible_actions, possible_param_values, belief=0.0):
        # This partition includes all belief states which are a tuple 
        # of one action in this possible_actions and one value of each 
        # param in possible_param_values which is a dict
        self.possible_actions = possible_actions
        self.possible_param_values = possible_param_values
        self.belief = belief

    def __str__(self):
        strForm = 'States: {' + ','.join(self.possible_actions) + '}'
        for param_name in self.possible_param_values :
            strParamValues = [str(v) for v in self.possible_param_values[param_name]]
            strForm = strForm + ' x {' + ','.join(strParamValues) + '}'
        strForm = strForm + ', Belief:' + str(self.belief)
        return strForm
        
    # Assumes elements of possible_actions and possible_param_values[key] for all keys
    # are atomic
    def __eq__(self, other) :
        if not checkLists(self.possible_actions, other.possible_actions) :
            return False
        elif not checkDicts(self.possible_param_values, other.possible_param_values) :
            return False
        elif not self.belief == other.belief :
            return False
        return True     
        
    # A partition matches with a system action and utterance if, for each attribute 
    # specified by them, including the action and its parameters, the partition allows 
    # only that value
    def match(self, system_action, utterance) :
        if system_action.name == 'confirm_action':
            if Knowledge.yes in utterance.extra_data :
               # User confirmed the action. So partition must exactly match what the systema action says
               if not len(self.possible_actions) == 1 or not system_action.referring_goal == self.possible_actions[0] :
                   return False          
               elif not system_action.referring_params == None :
                   for param_name in system_action.referring_params :
                       if param_name not in self.possible_param_values or len(self.possible_param_values[param_name]) != 1 or self.possible_param_values[param_name][0] != system_action.referring_params[param_name] :
                           return False
               return True
            else :                           
               # User did not confirm the action. So any partiton matches
               return True
        else :
            # All other system actions are information seeking. So partition has to match info in utterance
            if utterance.referring_goal != None :
                if len(self.possible_goals) != 1 or self.possible_goals[0] != utterance.referring_goal :
                    return False 
            if utterance.referring_params != None :
                for param_name in utterance.referring_params :
                    if param_name not in self.possible_param_values or len(self.possible_param_values[param_name]) != 1 or self.possible_param_values[param_name][0] != utterance.referring_params[param_name] :
                        return False           
            return True
            
# Simple tests to check syntax        
if __name__ == '__main__' :
    pa = ['a', 'b']
    pv = dict()
    pv['a'] = [1,2]
    pv['b'] = [3,4]
    p = Partition(pa, pv)
    q = Partition(pa, pv, 0)
    r = Partition(pa, pv, 1)
    s = Partition([], pv, 0)
    t = Partition(pa, dict(), 0)
    print str(p)
    print p == q
    print p == r
    print p == s
    print p == t        
    print "\n\n"
    
    m1 = SystemAction('confirm_action', 'searchroom', {'patient':'ray', 'location':'l3512'})
    m2 = SystemAction('repeat_goal')    
    u1 = Utterance('searchroom', {'patient':'ray', 'location':'l3512'})
    u2 = Utterance(None, None, [Knowledge.yes])
    u3 = Utterance(None, None, [Knowledge.no])
    p = [0,0,0,0,0,0,0,0]
    p[0] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512']})
    p[1] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512']})    
    p[2] = Partition(['searchroom', 'speak_t'], {'patient':['ray'], 'location':['l3512']})    
    p[3] = Partition(['searchroom'], {'patient':['ray', 'peter'], 'location':['l3512']})    
    p[4] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512', 'l3416']})        
    # Complete this test for match function
