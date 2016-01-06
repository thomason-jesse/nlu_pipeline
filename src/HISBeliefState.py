__author__ = 'aishwarya'

import itertools
from Partition import Partition

class HISBeliefState:

    def __init__(self, knowledge):

        self.knowledge = knowledge
        self.num_user_turns = 0

        # Number of possible actions the goal could be referring to
        self.num_goal_actions = len(knowledge.goal_actions) 
        self.num_utterance_actions = self.num_goal_actions + 1 # As action is utterance is allowed to be None  

        # n - Maximum possible number of parameters an action can have
        self.max_goal_params = len(knowledge.goal_params)     

        # Number of possible values each parameter can take
        self.num_goal_param_values = len(knowledge.goal_params_values) 

        # Create a partition with all possible states
        possible_actions = knowledge.goal_actions
        possible_param_values = dict()
        possible_param_values['patient'] = knowledge.goal_params_values
        possible_param_values['location'] = knowledge.goal_params_values
        possible_param_values['recipient'] = knowledge.goal_params_values
        complete_partition = Partition(possible_actions, possible_param_values, 1.0)
        self.partitions = [complete_partition]	
        
        # Hypotheses currently being tracked
        self.hypothesis_beliefs = None
        
        self.num_dialog_turns = 0
        
    def __str__(self) :
        return 'Partitions:\n\t' + '\n\t'.join([str(partition) for partition in self.partitions])

    def increment_dialog_turns(self) :
        self.num_dialog_turns += 1
        
    def get_matching_partitions(self, system_action, utterance) :
        matching_partitions = []
        for partition in self.partitions :
            if partition.match(system_action, utterance) :
                matching_partitions.append(partition)
        return matching_partitions
    
    # Warning: This function only creates matching partitions in the sense
    # that they match in goal and params. In the case of a confirm system
    # action, it does not check whether the user said yes or no    
    def make_all_matching_partitions(self, system_action, utterance, grounder) :
        #print 'In make_all_matching_partitions'
        #print '**********************************************'
        required_goal = None
        if system_action.referring_goal != None :
            required_goal = system_action.referring_goal
        elif utterance.referring_goal != None :
            required_goal = utterance.referring_goal    
        required_params = dict()
        if system_action.referring_params != None :
            for param_name in system_action.referring_params :
                if param_name in required_params :
                    required_params[param_name] = required_params[param_name] + system_action.referring_params[param_name]
                else :
                    required_params[param_name] = [system_action.referring_params[param_name]]
        if utterance.referring_params != None :
            for param_name in utterance.referring_params :
                if param_name in required_params :
                    required_params[param_name] = required_params[param_name] + utterance.referring_params[param_name]
                else :
                    required_params[param_name] = [utterance.referring_params[param_name]]

        #print "required_goal = ", required_goal        # DEBUG
        #print "required_params = ", required_params    # DEBUG
        
        # Iterate over the partitions and split any partition that is a 
        # superset of what is required
        new_partitions = list()
        for partition in self.partitions :
            #print '#########################'
            #print str(partition)
            #print 'Superset : ', partition.is_superset(required_goal, required_params)
            #print 'Equal : ', partition.is_equal(required_goal, required_params)
            #print '#########################'
            if partition.is_superset(required_goal, required_params) and not partition.is_equal(required_goal, required_params) :
                #print 'Entered if'
                #print '#########################'
                # This is a partition to be split
                goal_split_partitions = list()
                if required_goal != None :  # Split based on goal
                    goal_split_partitions = partition.split_by_goal(required_goal, self.knowledge, grounder)
                else :
                    goal_split_partitions = [partition]

                if required_params != None :
                    param_value_pairs = list()
                    for param_name in required_params :
                        param_value_pairs = param_value_pairs + [(param_name, value) for value in required_params[param_name]]
                    #print "param_value_pairs = ", param_value_pairs, '\n' # DEBUG
                    if len(param_value_pairs) == 0 :
                        new_partitions = new_partitions + goal_split_partitions
                        continue
                    partitions_to_split = goal_split_partitions
                    resultant_partitions = list()
                    # For each (param_name, param_value), take all the splits
                    # you have already made for the current partition and
                    # split each one of them acc to this pair
                    for (param_name, param_value) in param_value_pairs :
                        resultant_partitions = list()
                        for partition in partitions_to_split :
                            resultant_partitions = resultant_partitions + partition.split_by_param(param_name, param_value, self.knowledge, grounder)
                        partitions_to_split = resultant_partitions
                        
                    new_partitions = new_partitions + resultant_partitions
                else :
                    # There were no params to split on. So only add the 
                    # partitions obtained by splitting the goal 
                    new_partitions = new_partitions + goal_split_partitions
            else :
                # Either this partition is not a superset of the specified 
                # goal or params, or it is already an exact match of the 
                # required goal and params so don't split it
                new_partitions = new_partitions + [partition]    
        self.partitions = new_partitions
        #print '**********************************************'

    # Perform belief monitoring update using the system action and n best 
    # parses
    def update(self, system_action, n_best_utterances, grounder) :
        #print 'In update'
        #print '----------------------------------------------'
        #print str(system_action)
        for utterance in n_best_utterances :
            #print str(utterance)
            # Check whether there are partitions that exactly match the 
            # system action-utterance pair
            matching_partitions = self.get_matching_partitions(system_action, utterance)
            #print 'len(matching_partitions) = ', len(matching_partitions)
            if len(matching_partitions) == 0 :
                # if there are no matching partitions, split all partitions
                # that are supersets of the goal and param values of this 
                # pair to obtain matching partitions
                self.make_all_matching_partitions(system_action, utterance, grounder)
            #else :
                #for partition in matching_partitions :
                    #print str(partition)
        #print '----------------------------------------------'
        hypothesis_beliefs = dict()
        
        #print 'Updation'
        for partition in self.partitions :
            for utterance in n_best_utterances :
                #print '---------------------------------'
                #print str(partition)
                #print str(utterance)
               
                hypothesis = (partition, utterance)
                obs_prob = utterance.parse_prob # Pr(o'/u)
                #print 'obs_prob = ', obs_prob
                type_prob = self.knowledge.action_type_probs[system_action.action_type][utterance.action_type] # Pr(T(u)/T(m))
                #print 'type_prob = ', type_prob 
                param_match_prob = int(utterance.match(partition, system_action)) # Pr(M(u)/p,m)
                #print 'param_match_prob = ', param_match_prob 
                #print '---------------------------------'
                hypothesis_beliefs[hypothesis] = obs_prob * type_prob * param_match_prob * partition.belief 
                    # b(p',u') = k * Pr(o'/u) * Pr(T(u)/T(m)) * Pr(M(u)/p,m) * b(p)
            hypothesis = (partition, '-OTHER-')
            obs_prob = self.knowledge.obs_by_non_n_best_prob
            match_prob = self.knowledge.non_n_best_match_prob
            hypothesis_beliefs[hypothesis] = obs_prob * match_prob * partition.belief 
        
        # Remove invalid beliefs
        self.remove_invalid(grounder)
        
        # Normalize beliefs
        sum_hypothesis_beliefs = 0.0
        for hypothesis in hypothesis_beliefs.keys() :
            sum_hypothesis_beliefs += hypothesis_beliefs[hypothesis]
                
        partitionwise_sum = dict()
        #print 'Hypotheses - '
        for (partition, utterance) in hypothesis_beliefs.keys() :
           hypothesis_beliefs[(partition, utterance)] /= sum_hypothesis_beliefs
           #print '---------------------------------'
           #print str(partition)
           #print str(utterance)
           #print 'Belief = ', hypothesis_beliefs[(partition, utterance)]
           #print '---------------------------------'
           if partition not in partitionwise_sum :
               partitionwise_sum[partition] = hypothesis_beliefs[(partition, utterance)]
           else : 
               partitionwise_sum[partition] += hypothesis_beliefs[(partition, utterance)]
           
        # Reset partition beliefs for next round
        # b(p) = \sum_u b(p,u)
        for partition in self.partitions :
            partition.belief = partitionwise_sum[partition]

        self.hypothesis_beliefs = hypothesis_beliefs
        
    def remove_invalid(self, grounder) :
        if self.hypothesis_beliefs is None :
            return
        
        invalid_beliefs = set()  
        invalid_partitions = set()  
        for (partition, utterance) in self.hypothesis_beliefs.keys() :
            if not partition.is_valid(self.knowledge, grounder) :
                invalid_beliefs.add((partition, utterance))
                invalid_partitions.add(partition)
            elif type(utterance) == 'Utterance' and not utterance.is_valid(self.knowledge, grounder) :
                invalid_beliefs.add((partition, utterance))
                
        for key in invalid_beliefs :
            if key in self.hypothesis_beliefs :
                del self.hypothesis_beliefs[key]
        
        for partition in invalid_partitions :
            if partition in self.partitions : 
                self.partitions.remove(partition)    
        
            
            
