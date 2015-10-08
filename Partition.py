__author__ = 'aishwarya'

from Utils import *
from Knowledge import Knowledge

# For testing only
from SystemAction import SystemAction
from Utterance import Utterance

class Partition:

    def __init__(self, possible_goals=None, possible_param_values=None, belief=0.0):
        # This partition includes all belief states which are a tuple 
        # of one action in this possible_goals and one value of each 
        # param in possible_param_values which is a dict
        self.possible_goals = possible_goals
        self.possible_param_values = possible_param_values
        self.belief = belief

    def __str__(self):
        strForm = 'States: {' + ','.join(self.possible_goals) + '}'
        for param_name in self.possible_param_values :
            strParamValues = [str(v) for v in self.possible_param_values[param_name]]
            strForm = strForm + ' x {' + ','.join(strParamValues) + '}'
        strForm = strForm + ', Belief:' + str(self.belief)
        return strForm
        
    # Assumes elements of possible_goals and possible_param_values[key] for all keys
    # are atomic
    def __eq__(self, other) :
        if not checkLists(self.possible_goals, other.possible_goals) :
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
            if utterance.extra_data is not None and Knowledge.yes in utterance.extra_data :
               # User confirmed the action. So partition must exactly match what the systema action says
               if not len(self.possible_goals) == 1 or not system_action.referring_goal == self.possible_goals[0] :
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
    
    # Check whether this partition contains all states having some goal 
    # and params        
    def is_superset(self, required_goal, required_params) :
        if required_goal != None and required_goal not in self.possible_goals :
            # We want a specific goal and that is not in this partition
            return False
        elif required_params != None :
            for param_name in required_params :
                if param_name not in self.possible_param_values :
                    # This should not ideally ever happen
                    print "\nCheck partitions.py line 77 as this should not happen\n"
                    return False
                else :
                    if not set(required_params[param_name]).issubset(set(self.possible_param_values[param_name])) :
                        # The values we want for this param are not a subset 
                        # of the values allowed by the partition
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
    p = [0,0,0,0,0,0]
    p[0] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512']})
    p[1] = Partition(['searchroom'], {'patient':[], 'location':[]})    
    p[2] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512']})    
    p[3] = Partition(['searchroom', 'speak_t'], {'patient':['ray'], 'location':['l3512']})    
    p[4] = Partition(['searchroom'], {'patient':['ray', 'peter'], 'location':['l3512']})    
    p[5] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512', 'l3416']})        
    
    for pp in p :
      print '\n'.join([str(pp), str(m1), str(u2)])
      print pp.match(m1, u2)
      print '\n'
      print '\n'.join([str(pp), str(m1), str(u3)])
      print pp.match(m1, u3)
      print '\n'
      print '\n'.join([str(pp), str(m2), str(u1)])
      print pp.match(m1, u2)
      print '\n'
        
    
