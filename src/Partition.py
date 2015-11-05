__author__ = 'aishwarya'

from Utils import *

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
        #strForm = strForm + 'Partition prob:' + str(self.reaching_prob) + '\n'
        return strForm
        
    def __hash__(self):
        dict_tuple = ()
        if self.possible_param_values is not None :
            dict_tuple = tuple([(k, tuple(v)) for (k, v) in self.possible_param_values.items()])
        return hash((tuple(self.possible_goals), dict_tuple, self.belief, self.reaching_prob))
        
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
        if system_action.action_type == 'confirm_action' and utterance.action_type == 'deny' :
            return True
            
        # Partition should match info in system action
        if not len(self.possible_goals) == 1 or not system_action.referring_goal == self.possible_goals[0] :
            return False          
        elif not system_action.referring_params == None :
            for param_name in system_action.referring_params :
                if param_name not in self.possible_param_values or len(self.possible_param_values[param_name]) != 1 or self.possible_param_values[param_name][0] != system_action.referring_params[param_name] :
                    return False
        return True
            
        # Partition has to match info in utterance
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
                        #print param_name, " values not a subset"
                        #print "Available values: ", self.possible_param_values[param_name]
                        #print "Queried value: ", required_params[param_name]
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
        if new_goal not in self.possible_goals or len(self.possible_goals) == 1 :
            return [self]
        else :
            p1 = Partition([new_goal], self.possible_param_values)
            split_prob = knowledge.partition_split_goal_probs[new_goal]
            p1.reaching_prob = self.reaching_prob * split_prob
            p1.belief = self.belief * split_prob
            other_goals = [goal for goal in self.possible_goals if goal != new_goal]
            p2 = Partition(other_goals, self.possible_param_values)
            p2.reaching_prob = self.reaching_prob * (1 - split_prob)
            p2.belief = self.belief * (1 - split_prob)
            return [p1, p2]

    def split_by_param(self, split_param_name, split_param_value, knowledge) :
        if self.possible_param_values == None or \
        split_param_name not in self.possible_param_values or \
        split_param_value not in self.possible_param_values[split_param_name] or \
        len(self.possible_param_values[split_param_name]) == 1 :
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
            p1.belief = self.belief * split_prob
            p2.reaching_prob = self.reaching_prob * (1 - split_prob)
            p2.belief = self.belief * (1 - split_prob)
            return [p1, p2]
            
