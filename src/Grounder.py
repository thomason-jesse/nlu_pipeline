__author__ = 'aishwarya'

# This is supposed to convert parses (instances fo SemanticNode) to
# grounded hypotheses (instances of Utterance)

# NOTE: In our domain, you never have more than one lambda variable in a parse
# While the grounder doesn't explicitly assume that this is true, it hasn't
# really been tested for cases where this isn't
# It should work fine provided lambda variable names are not reused for 
# different variables with different scopes

# This defines grounding probability of parse p as probability that 
# grounding g satisfies p
# For example, if p = \lambda x:e.(ball(x)). If the ball classifier says
# item_1 is a ball with probability c1 and item_2 is a ball with probability 
# c2, then the grounding probability of item_1 for p is c1 and that of 
# item_2 for p is c2 regardless of whether c1 + c2 > 1 
# So it is possible for multiple groundings of p to have grounding probability 1
# Grounding probabilities have to be normalized before they can be used
# as hypothesis probabilities

import numpy, itertools, random

from utils import *
from Action import Action

class Grounder :
    # Expected inputs - 
    #   ontology - instance of Ontology
    #   perception_module - instance of ActiveLearner (optional)
    #   kb_predicates - A dict whose keys are pedicate names and values 
    #       are a list of entities or entity tuples for which the predicate holds
    #   classifier_predicates - A list of predicates to be grounded using 
    #       classifiers. This should be a subset of perception_module.label_names
    def __init__(self, ontology, perception_module=None, kb_predicates=None, classifier_predicates=None) :
        # Basic consistency check - classifier_predicates should be a subset of perception_module.label_names
        if classifier_predicates is not None and len(classifier_predicates) > 0:
            if perception_module is None or perception_module.label_names is None :
                raise RuntimeError('classifier_predicates is not empty but no perception_module passed')
            else :
                labels_without_classifiers = [label_name for label_name in classifier_predicates if label_name not in perception_module.label_names]
                if len(labels_without_classifiers) > 0 :
                    raise RuntimeError('No classifiers in perception_module for labels ' + str(labels_without_classifiers))
                    
        self.ontology = ontology
        self.perception_module = perception_module
        self.kb_predicates = kb_predicates
        self.classifier_predicates = classifier_predicates
    
    def set_domain_of_discourse(self, domain_of_discourse):
        self.perception_module.set_domain_of_discourse(domain_of_discourse)
            
    def predicate_holds(self, predicate, argument) :
        print 'In predicate_holds with predicate = ', predicate, ', argument = ', argument
        if self.kb_predicates is not None and predicate in self.kb_predicates :
            if argument in self.kb_predicates[predicate] :
                return True
            else :
                return False
        elif self.classifier_predicates is not None and predicate in self.classifier_predicates :
            if not self.perception_module.has_classifier(predicate) :
                return False
            elif not argument.startswith('item_') :
                # Only things which can be tested by classifiers are those named item_i
                return False
            else :
                item_idx = int(argument.split('_')[1]) 
                vgg_feature = self.test_object_features[item_idx, :]
                label_array = self.perception_module.get_label(vgg_feature, predicate)
                if label_array[0] > 0 :
                    return True
                else :
                    return False
        else :
            return False
            
    
    # Returns all possible ontological assignments to lambdas of a given type
    def get_assignments_for_type(self, type):
        return [i for i in range(0, len(self.ontology.preds)) if self.ontology.entries[i] == type]    
    
    # Expects an instance of class SemanticNode    
    # Returns (groundings, lambda_assignments) where 
    #   groundings - list of (grounding, log_prob)
    #                where grounding is either a string or Action object   
    #   lambda_assignments = dict whose key is lambda names and value is 
    #                        a list of strings that satisfy downstream constraints      
    def ground_semantic_node(self, parse) :
        # A parse with no children can be directly grounded
        if parse.children is None :
            if parse.idx is not None :
                # No lambda assignment involved. Return the predicate at the node
                # This is deterministic, so prob of grounding is 1 and log prob is 0
                return ([(self.ontology.preds[parse.idx], 0)], {})
            elif parse.is_lambda :
                # This node is a lambda variable. 
                # Also return all possible valid assignments
                type_consistent_assignments = self.get_assignments_for_type(parse.type)
                ont_preds = [self.ontology.preds[idx] for idx in type_consistent_assignments]
                assignments_with_log_probs = [(assignment, 0) for assignment in ont_preds]
                lambda_assignments = dict()
                lambda_assignments[parse.lambda_name] = assignments_with_log_probs
                return ([], lambda_assignments)
            else :
                # Invalid parse
                print 'Warning: Leaf node is not ontology atom or lambda variable!'
                return ([], {})   
        
        else :
            # The node has children. They need to be grounded before this
            # node can be
            child_groundings = [self.ground_semantic_node(child) for child in parse.children]
            #print [(str(grounding), prob) for (grounding, prob) in child_groundings]
            
            if parse.is_lambda and parse.is_lambda_instantiation :
                # This corresponds to a lambda x.<type> function. It should
                # have a single child because things which combine predicates
                # such as 'and' operations get their own nodes
                if len(parse.children) != 1 :
                    # Invalid parse
                    print 'Warning: Lambda node should have only one child but has ' + str(len(parse.children)) + ' children!'
                    return ([], {})   
                (groundings, lambda_assignments) = child_groundings[0]
                if parse.lambda_name not in lambda_assignments :
                    # Invalid parse
                    print 'Warning: No downstream use of lambda variable ' + str(parse.lambda_name) + '!'
                    return ([], {})   
                    
                #print 'lambda x node'
                #print 'groundings = ', groundings    
                #print 'lambda_assignments = ', lambda_assignments
                #x = raw_input()
                groundings = lambda_assignments[parse.lambda_name]
                lambda_assignments.pop(parse.lambda_name, None) 
                    # This lambda variable should be removed because it is no longer in scope in upper nodes
                return (groundings, lambda_assignments)
                    
            elif self.ontology.types[parse.return_type] == 'a' :
                # Grounds to an action. Create an Action object for
                # each possible grounding
                
                # Note: To get UNKs in actions, replace empty child 
                # grounding lists with [(UNK, 0)]
                child_groundings_without_assignments = [groundings for (groundings, lambda_assignments) in child_groundings]
                    # Lambda assignments are no longer needed
                #print 'child_groundings_without_assignments = ', child_groundings_without_assignments
                    
                # Take Cartesian product of individual parameter groundings to
                # get all possible parameter combinations plausible for the parse
                possible_param_tuples = list(itertools.product(*child_groundings_without_assignments))
                #print 'possible_param_tuples = ', possible_param_tuples
                
                action_name = self.ontology.preds[parse.idx]
                actions = list()
                for param_combination in possible_param_tuples :
                    #print 'param_combination = ', param_combination
                    action = Action(action_name)
                    params = [param for (param, log_prob) in param_combination]
                    #print 'param_combination = ', param_combination 
                    #print 'params = ', params
                    action.params = params
                    log_probs = [log_prob for (param, log_prob) in param_combination]
                    #print 'log_probs = ', log_probs
                    # Assume prob of params is independent => action prob
                    # is product of param probs => action log prob is sum 
                    # of product log probs
                    action_log_prob = sum(log_probs)
                    actions.append((action, action_log_prob))
                return (actions, {})
    
            elif self.ontology.preds[parse.idx] == 'equals' :
                raise RuntimeError('Grounding of \'equals\' predicate has not been implemented')
                
            elif self.ontology.preds[parse.idx] == 'and' :
                if self.ontology.types[parse.return_type] != 't' :
                    raise RuntimeError('Grounding of \'and\' except as a combiner of <e,t> predicates has not been implemented') 
                    
                # Child groundings are irrelevant. We only care about lambda assignments
                all_lambda_assignments = [lambda_assignments for (groundings, lambda_assignments) in child_groundings]
                
                # Will only consider assignments to variables that are grounded in all children
                common_lambda_variables = set([key for key in all_lambda_assignments[0].keys() if len(all_lambda_assignments[0][key]) > 0])
                for lambda_assignments in all_lambda_assignments :
                    possible_lambda_variables = [key for key in lambda_assignments.keys() if len(lambda_assignments[key]) > 0]
                    common_lambda_variables = common_lambda_variables.intersection(possible_lambda_variables)
                    
                resultant_lambda_assignments = dict()
                for variable in common_lambda_variables :
                    assignments = [lambda_assignments[variable] for lambda_assignments in all_lambda_assignments]
                    assignments_without_probs =  [[candidate for (candidate, log_prob) in assignment] for assignment in assignments]
                    common_candidates = set(assignments_without_probs[0]).intersection(*assignments_without_probs)
                    log_probs = dict()
                    for assignment in assignments :
                        for (candidate, log_prob) in assignment :
                            if candidate in common_candidates :
                                if candidate not in log_probs :
                                    log_probs[candidate] = 0
                                log_probs[candidate] = log_probs[candidate] + log_prob
                                    # Basically an independence assumption
                                    # Probability that item1 is a ball and red is the 
                                    # product of the probability that it is a ball and 
                                    # the probability that it is red
                    resultant_lambda_assignments = [(candidate, log_probs[candidate]) for candidate in common_candidates]
                return ([], resultant_lambda_assignments)

            elif self.ontology.preds[parse.idx] == 'or' :
                raise RuntimeError('Grounding of \'or\' predicate has not been implemented')
                
            elif self.ontology.preds[parse.idx] == 'not' :
                raise RuntimeError('Grounding of \'not\' predicate has not been implemented')
                
            elif self.ontology.preds[parse.idx] == 'or' :
                raise RuntimeError('Grounding of \'or\' predicate has not been implemented')

            elif self.ontology.preds[parse.idx] in ['a', 'the'] :
                print 'Warning: \'a\' and \'the\' are treated equally for dialog purposes!' 
                #x = raw_input()
                if len(parse.children) != 1 :
                    # Invalid parse
                    print 'Warning: \'a\' or \'the\' nodes should have only one child but has ' + str(len(parse.children)) + ' children!'
                    return ([], {})   
                #print 'For a/the, child_groundings = ', child_groundings
                #x = raw_input()
                random_child = random.choice(child_groundings)
                return (random_child[0], {})
                    
            elif self.classifier_predicates is not None and self.ontology.preds[parse.idx] in self.classifier_predicates :
                # WARNING: This assumes that you will only do is_red(x) 
                # never is_red(item_1) to get the truth value
                
                if len(parse.children) != 1 :
                    # Invalid parse
                    print 'Warning: Classifier predicate node should have only one child but has ' + str(len(parse.children)) + ' children!'
                    return ([], {})  

                classifier_name = self.ontology.preds[parse.idx]
                if not self.perception_module.has_classifier(classifier_name) :
                    return ([], {})  
                    
                (groundings, lambda_assignments) = child_groundings[0]
                #print 'For classifier preds'
                #print 'groundings = ', groundings
                #print 'lambda_assignments = ', lambda_assignments
                
                resultant_lambda_assignments = dict()
                for variable in lambda_assignments.keys() :
                    possible_assignments = lambda_assignments[variable]
                    items = [assignment[0] for assignment in possible_assignments if assignment[0].startswith('item_')]
                        # Only atoms which are 'item_<item_num>' can be classified
                    #print 'items = ', items
                    if len(items) > 0 :
                        item_indices = [int(item.split('_')[1]) for item in items]
                        vgg_features = self.test_object_features[item_indices, :]
                        probs = self.perception_module.get_classifier_log_probs(vgg_features, classifier_name)    
                        resultant_lambda_assignments[variable] = zip(items, probs)
                    
                return ([], resultant_lambda_assignments)
                
            elif self.kb_predicates is not None and self.ontology.preds[parse.idx] in self.kb_predicates :
                predicate = self.ontology.preds[parse.idx]
                #print 'Checking for kb predicate : ', predicate
                if predicate not in self.kb_predicates :
                    return ([], {})  
                    
                true_tuples = self.kb_predicates[predicate]
                if len(true_tuples) == 0 :
                    return ([], {})  
                    
                if len(parse.children) != len(true_tuples[0]) :
                    print 'Warning: Predicate ' + str(predicate) + ' expects ' + str(len(true_tuples[0])) + ' child nodes but has ' + str(len(parse.children)) + '!'
                    return ([], {})  

                all_lambda_assignments = [lambda_assignments for (groundings, lambda_assignments) in child_groundings]
                num_children_with_lambda_assignments = len([lambda_assignments for lambda_assignments in all_lambda_assignments if len(lambda_assignments.keys()) > 0])
                if num_children_with_lambda_assignments > 1 :
                    raise RuntimeError('Grounder cannot handle KB predicates that need to resolve multiple lambda variables')
                #print 'all_lambda_assignments = ', all_lambda_assignments
                
                child_lambda_assignments = None 
                child_lambda_assignments_idx = None
                for (child_idx, (groundings, lambda_assignments)) in enumerate(child_groundings) :
                    if groundings is not None and len(groundings) > 0 :
                        groundings_without_probs = [grounding for (grounding, prob) in groundings]
                        true_tuples = [candidate_tuple for candidate_tuple in true_tuples if candidate_tuple[child_idx] in groundings_without_probs]
                    if lambda_assignments is not None and len(lambda_assignments.keys()) > 0 :
                        child_lambda_assignments = lambda_assignments
                        child_lambda_assignments_idx = child_idx
                #print 'child_lambda_assignments = ', child_lambda_assignments
                #print 'true_tuples = ', true_tuples
                        
                if len(true_tuples) == 0 or child_lambda_assignments is None :
                    #print 'len(true_tuples) == 0 or child_lambda_assignments is None'
                    return ([], {})  
                    
                allowed_vals = [candidate_tuple[child_lambda_assignments_idx] for candidate_tuple in true_tuples]
                #print 'allowed_vals = ', allowed_vals
                resultant_lambda_assignments = dict()
                for lambda_variable in child_lambda_assignments :
                    resultant_lambda_assignments[lambda_variable] = [(assignment, prob) for (assignment, prob) in child_lambda_assignments[lambda_variable] if assignment in allowed_vals]
                return ([], resultant_lambda_assignments)
                
            else :
                raise RuntimeError('Node did not satisfy any groundable condition')
