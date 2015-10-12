__author__ = 'aishwarya'

from Utils import *
from Knowledge import Knowledge

# For testing only
from SystemAction import SystemAction
from Utterance import Utterance

class Partition:

    def __init__(self, possible_goals=None, possible_param_values=None, belief=0.0, reaching_prob=1.0):
        # This partition includes all belief states which are a tuple 
        # of one action in this possible_goals and one value of each 
        # param in possible_param_values which is a dict
        self.possible_goals = possible_goals
        self.possible_param_values = possible_param_values
        self.belief = belief
        self.reaching_prob = reaching_prob      # Pr(this partition/root) = 
                                                # Pr(this partition / its parent)Pr(its parent/its grandparent)...

    def __str__(self):
        strForm = 'States: {' + ','.join(self.possible_goals) + '}\n'
        for param_name in self.possible_param_values :
            strParamValues = [str(v) for v in self.possible_param_values[param_name]]
            strForm = strForm + param_name + ': {' + ','.join(strParamValues) + '}\n'
        strForm = strForm + 'Belief:' + str(self.belief) + '\n'
        strForm = strForm + 'Partition prob:' + str(self.reaching_prob) + '\n'
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
        elif not self.reaching_prob == other.reaching_prob :
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
            print "Goal not present"
            # We want a specific goal and that is not in this partition
            return False
        elif required_params != None :
            for param_name in required_params :
                if param_name not in self.possible_param_values :
                    print param_name, " not present in call to is_superset"
                    # This should not ideally ever happen
                    print "\nCheck partitions.py for this line as this should not happen\n"
                    return False
                else :
                    if not set(required_params[param_name]).issubset(set(self.possible_param_values[param_name])) :
                        print param_name, " values not a subset"
                        print "Available values: ", self.possible_param_values[param_name]
                        print "Queried value: ", required_params[param_name]
                        # The values we want for this param are not a subset 
                        # of the values allowed by the partition
                        return False
        return True
    
    # Check whether this partition has only the goal and param values indicated
    def is_equal(self, required_goal, required_params) :
        if required_goal != None : # We want a specific goal
            if required_goal not in self.possible_goals : 
                # Required goal is not in this partition
                return False
            elif len(self.possible_goals) > 1 :
                # Required goal is not the only goal in this partition
                return False
        elif required_params != None :
            # We have some specified params
            for param_name in required_params :
                if param_name not in self.possible_param_values :
                    print param_name, " not present in call to is_equal"
                    # This should not ideally ever happen
                    print "\nCheck partitions.py for this line as this should not happen\n"
                    return False
                else :
                    if not set(required_params[param_name]) == set(self.possible_param_values[param_name]) :
                        # The values we want for this param are the same  
                        # as the values allowed by the partition
                        return False
        return True
            
    def split_by_goal(self, new_goal, knowledge) :
        if new_goal not in self.possible_goals :
            return [self]
        else :
            p1 = Partition([new_goal], self.possible_param_values)
            split_prob = knowledge.partition_split_goal_probs[new_goal]
            p1.reaching_prob = self.reaching_prob * split_prob
            other_goals = [goal for goal in self.possible_goals if goal != new_goal]
            p2 = Partition(other_goals, self.possible_param_values)
            p2.reaching_prob = self.reaching_prob * (1 - split_prob)
            return [p1, p2]

    def split_by_param(self, split_param_name, split_param_value, knowledge) :
        if self.possible_param_values == None or split_param_name not in self.possible_param_values or split_param_value not in self.possible_param_values[split_param_name] :
            return [self]
        else :
            p1_param_values = dict()
            p2_param_values = dict()
            for param_name in self.possible_param_values :
                if param_name != split_param_name :
                    p1_param_values[param_name] = self.possible_param_values[param_name]
                    p2_param_values[param_name] = self.possible_param_values[param_name]
                else :
                    p1_param_values[param_name] = [split_param_value]
                    p2_param_values[param_name] = [param_value for param_value in self.possible_param_values[split_param_name] if param_value != split_param_value]
            p1 = Partition(self.possible_goals, p1_param_values)
            p2 = Partition(self.possible_goals, p2_param_values)
            split_prob = knowledge.partition_split_param_probs[split_param_name][split_param_value]
            p1.reaching_prob = self.reaching_prob * split_prob
            p2.reaching_prob = self.reaching_prob * (1 - split_prob)
            return [p1, p2]
            
# Simple tests to check syntax        
if __name__ == '__main__' :
    pa = ['searchroom', 'remind', 'askperson']
    pv = dict()
    pv['location'] = ['3502', '3414b']
    pv['patient'] = ['peter', 'ray']
    p = Partition(pa, pv)
    knowledge = Knowledge()
    print 1, [str(pp) for pp in p.split_by_goal('c', knowledge)], '\n'
    l = p.split_by_goal('remind', knowledge)
    print 2, [str(pp) for pp in l], '\n'
    print 3, [str(pp) for pp in l[0].split_by_param('abc', '3502', knowledge)], '\n'
    print 4, [str(pp) for pp in l[0].split_by_param('location', 'xyz', knowledge)], '\n'
    print 5, [str(pp) for pp in l[0].split_by_param('location', '3502', knowledge)], '\n'
    
    #q = Partition(pa, pv, 0)
    #r = Partition(pa, pv, 1)
    #s = Partition([], pv, 0)
    #t = Partition(pa, dict(), 0)
    #print str(p)
    #print p == q
    #print p == r
    #print p == s
    #print p == t        
    #print "\n\n"
    
    #m1 = SystemAction('confirm_action', 'searchroom', {'patient':'ray', 'location':'l3512'})
    #m2 = SystemAction('repeat_goal')    
    #u1 = Utterance('searchroom', {'patient':'ray', 'location':'l3512'})
    #u2 = Utterance(None, None, [Knowledge.yes])
    #u3 = Utterance(None, None, [Knowledge.no])
    #p = [0,0,0,0,0,0]
    #p[0] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512']})
    #p[1] = Partition(['searchroom'], {'patient':[], 'location':[]})    
    #p[2] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512']})    
    #p[3] = Partition(['searchroom', 'speak_t'], {'patient':['ray'], 'location':['l3512']})    
    #p[4] = Partition(['searchroom'], {'patient':['ray', 'peter'], 'location':['l3512']})    
    #p[5] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512', 'l3416']})        
    
    #for pp in p :
      #print '\n'.join([str(pp), str(m1), str(u2)])
      #print pp.match(m1, u2)
      #print '\n'
      #print '\n'.join([str(pp), str(m1), str(u3)])
      #print pp.match(m1, u3)
      #print '\n'
      #print '\n'.join([str(pp), str(m2), str(u1)])
      #print pp.match(m1, u2)
      #print '\n'
        
    
