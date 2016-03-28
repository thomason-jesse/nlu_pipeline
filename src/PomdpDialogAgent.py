__author__ = 'aishwarya'

import sys, random, math, copy, re, numpy, itertools, traceback
from HISBeliefState import HISBeliefState
from SummaryState import SummaryState
from SystemAction import SystemAction
from Utterance import Utterance
from DialogAgent import DialogAgent
from TemplateBasedGenerator import TemplateBasedGenerator
from Knowledge import Knowledge
from utils import *

# This dialog agent maintains a much richer state and hence has to 
# process system actions and user responses more elaborately. It extends
# StaticDialogAgent primarily to reuse code
class PomdpDialogAgent(DialogAgent) :

    def __init__(self, parser, grounder, policy, u_input, output, parse_depth=10):
        DialogAgent.__init__(self, parser, grounder, policy, u_input, output, parse_depth)
        self.policy = policy

        # To convert system actions to understandable English
        self.response_generator = TemplateBasedGenerator()
        # For typechecking information
        self.knowledge = Knowledge()    

        self.state = HISBeliefState(self.knowledge)  
        self.previous_system_action = SystemAction('repeat_goal')  
        self.n_best_utterances = None

        self.first_turn = True
        
        # To store data for retraining the parser
        self.retrain_parser = False     # Retrain parser at the end of 
                                        # every dialogue
        self.parser_train_data = None
        self.max_prob_user_utterances = None
        self.max_parses_examined_in_retraining = 10
        
        
        # Logging for future use
        self.final_action_log = None
        self.lexical_addition_log = None
        # Pickle and store system action and user response at each turn
        # with some other information useful for retraining
        self.dialog_objects_logfile = None 
        self.dialog_objects_log = None
        self.cur_turn_log = None
        self.log_header = 'pomdp'
        
    # Returns True if the dialog terminates on its own and False if the u
    # user entered stop
    def run_dialog(self) :
        self.state = HISBeliefState(self.knowledge)
        self.parser_train_data = dict()
        self.max_prob_user_utterances = list()
        self.dialog_objects_log = list()
        
        print 'Belief state: ', str(self.state) 
        summary_state = SummaryState(self.state)
        (dialog_action, dialog_action_arg) = self.policy.get_initial_action(summary_state)    
        
        while True :
            self.state.increment_dialog_turns()
            if dialog_action == 'take_action' :
                # Submit the action given by the policy
                action = dialog_action_arg
                output = self.response_generator.get_action_sentence(action, self.final_action_log)
                self.output.say(output + " Was this the correct action?")
                response = self.input.get().lower()  
                if response == '<ERROR/>' :
                    # Benefit of doubt - assume the action was correct
                    self.policy.update_final_reward(self.knowledge.correct_action_reward)
                    complete_log_object = (self.log_header, self.dialog_objects_log, action, True, self.parser_train_data)
                    if self.dialog_objects_logfile is not None :
                        save_obj_general(complete_log_object, self.dialog_objects_logfile)
                    return False  
                if response.lower() == 'y' or response.lower() == 'yes' :
                    self.policy.update_final_reward(self.knowledge.correct_action_reward)
                    self.cur_turn_log = ['take_action', None]
                    self.dialog_objects_log.append(self.cur_turn_log)
                    complete_log_object = (self.log_header, self.dialog_objects_log, action, True, self.parser_train_data)
                    if self.dialog_objects_logfile is not None :
                        save_obj_general(complete_log_object, self.dialog_objects_logfile)
                    if self.retrain_parser :
                        self.train_parser_from_dialogue(action)
                else :
                    self.cur_turn_log = ['take_action', None]
                    self.dialog_objects_log.append(self.cur_turn_log)
                    complete_log_object = (self.log_header, self.dialog_objects_log, action, False, self.parser_train_data)
                    if self.dialog_objects_logfile is not None :
                        save_obj_general(complete_log_object, self.dialog_objects_logfile)
                    self.policy.update_final_reward(self.knowledge.wrong_action_reward)
                    return False
                return True
            else :
                self.previous_system_action = dialog_action_arg
                print 'System action - '
                print str(self.previous_system_action)
                # Take the dialog action
                response = self.dialog_action_functions[self.dialog_actions.index(dialog_action)]()
                self.cur_turn_log = [self.previous_system_action, response]
                if response == '<ERROR/>' :
                    complete_log_object = (self.log_header, self.dialog_objects_log, None, False, self.parser_train_data)
                    if self.dialog_objects_logfile is not None :
                        save_obj_general(complete_log_object, self.dialog_objects_logfile)
                    return False
                if response == 'stop' :
                    complete_log_object = (self.log_header, self.dialog_objects_log, None, False, self.parser_train_data)
                    if self.dialog_objects_logfile is not None :
                        save_obj_general(complete_log_object, self.dialog_objects_logfile)
                    self.policy.update_final_reward(self.knowledge.wrong_action_reward)
                    return False
                # Belief monitoring - update belief based on the response
                self.update_state(response)
                print 'Belief state: ', str(self.state) # DEBUG
                
                reward = self.knowledge.per_turn_reward
                self.dialog_objects_log.append(self.cur_turn_log)
                
                summary_state = SummaryState(self.state)
                # Get the next action from the policy
                (dialog_action, dialog_action_arg) = self.policy.get_next_action(reward, summary_state)
                #print 'dialog_action = ', dialog_action # DEBUG
                #print str(dialog_action_arg)            # DEBUG
            self.first_turn = False
        
    # Request the user to state/repeat their goal
    def request_user_initiative(self):
        if self.first_turn :
            self.previous_system_action.extra_data = ['first']
        output = self.response_generator.get_sentence(self.previous_system_action)
        print 'output = ', output
        self.output.say(output)
        response = self.input.get()
        if 'full' in self.parser_train_data :
            self.parser_train_data['full'].append(response)
        else :
            self.parser_train_data['full'] = [response]
        return response

    # Request a missing action parameter
    def request_missing_param(self):
        output = self.response_generator.get_sentence(self.previous_system_action)
        self.output.say(output)
        response = self.input.get()
        param_name = self.previous_system_action.extra_data[0]
        if param_name in self.parser_train_data :
            self.parser_train_data[param_name].append(response)
        else :
            self.parser_train_data[param_name] = [response]
        return response

    # Parse the user response and update belief state
    def update_state(self, response) :
        # get n best parses for confirmation
        n_best_parses = self.get_n_best_parses(response)

        # Create an n-best list of Utterance objects along with probabiltiies 
        # obtained from their confidences
        self.get_n_best_utterances_from_parses(n_best_parses)
        #print 'Created utterances'
        
        if len(self.n_best_utterances) > 0 :
            # Update the belief state
            #print 'Previous system action - ', str(self.previous_system_action) # DEBUG
            #print 'Utterances - ', '\n'.join([str(utterance) for utterance in self.n_best_utterances])
            self.state.update(self.previous_system_action, self.n_best_utterances, self.grounder)
            print 'Updated'

    # Confirm a (possibly partial) action
    def confirm_action(self):
        # with reverse parsing, want to confirm(a.name(a.params))
        # for now, just use a.name and a.params raw
        output = self.response_generator.get_sentence(self.previous_system_action)
        self.output.say(output)                        
        response = self.input.get()
        return response

    # Converts an object of type Action to type Utterance assuming it is
    # informing the system of goal and param details
    def convert_action_to_utterance(self, action) :
        utterance = None
        if action is not None :
            goal = action.name
            if action.params is None :
                utterance = Utterance('inform_full', goal)
            else :
                params = dict()
                for (idx, param_val) in enumerate(action.params) :
                    param_name = self.knowledge.param_order[goal][idx]
                    if param_val != 'UNK_E' :
                        params[param_name] = param_val
                utterance = Utterance('inform_full', goal, params)
        return utterance

    def create_utterances_of_parse(self, parse) :
        parse = parse.node
        top_level_category = self.parser.lexicon.categories[parse.category]
        
        #print 'In create_utterances_of_parse with', self.parser.print_parse(parse, show_category=True)
        #print '\n\n'

        # Confirmation (affirm/deny/none)
        if top_level_category == 'C' :
            if parse.idx == self.parser.ontology.preds.index('yes'):
                #print 'Is affirm'
                return [Utterance('affirm')]
            elif parse.idx == self.parser.ontology.preds.index('no'):
                #print 'Is deny'
                return [Utterance('deny')]
            elif parse.idx == self.parser.ontology.preds.index('none'):
                #print 'Got a none response'
                goal = self.previous_system_action.referring_goal
                if self.previous_system_action.extra_data is not None and len(self.previous_system_action.extra_data) >= 1:
                    param_name = self.previous_system_action.extra_data[0]
                else :
                    return []
                params = dict()
                params[param_name] = None
                utterance = Utterance('inform_param', goal, params)      
                #print '\n'
                return [utterance]
            else :
                return []
        
        # Complete action with goal and params
        elif top_level_category == 'M' :
            try:
                #print 'Trying to get an action'
                p_action = self.get_action_from_parse(parse)
            except SystemError:
                p_action = None

            if p_action is not None :
                #print 'Got an action. Will convert and return'
                return [self.convert_action_to_utterance(p_action)]
            else :
                return []
        
        # To be grounded as param value
        elif top_level_category == 'NP' :
            if parse.is_lambda and parse.is_lambda_instantiation :
                # Lambda headed parse - unlikely to give a valid param
                # value
                return []
            #print 'Trying to ground as param value'
            try :
                #print "Grounding ", self.parser.print_parse(parse)
                g = self.grounder.groundSemanticNode(parse, [], [], [])
                answers = self.grounder.grounding_to_answer_set(g)
                #print 'answers = ', answers
                #print '--------------------------------'
                utterances = []
                if type(answers) == 'str' :
                    answers = [answers]
                for answer in answers :
                    goal = self.previous_system_action.referring_goal
                    params = dict()
                    #print 'self.previous_system_action.extra_data = ', self.previous_system_action.extra_data
                    if self.previous_system_action.action_type == 'request_missing_param' and self.previous_system_action.extra_data is not None and len(self.previous_system_action.extra_data) == 1 :
                        param_name = self.previous_system_action.extra_data[0]
                        params[param_name] = answer
                        utterance = Utterance('inform_param', goal, params)      
                        utterances.append(utterance)
                    else :
                        for param_name in self.knowledge.goal_params :
                            params_copy = copy.deepcopy(params)
                            params_copy[param_name] = answer
                            utterance = Utterance('inform_param', goal, params_copy)      
                            utterances.append(utterance)
                return utterances
            except TypeError as e :
                error = str(sys.exc_info()[0])
                if self.error_log is not None :
                    self.error_log.write(traceback.format_exc() + '\n\n\n')
                    self.error_log.flush() 
                print traceback.format_exc()
                return []

    def remove_invalid_utterances(self, utterances, grounder) :
        invalid_utterances = set()
        for utterance in utterances :
            if not utterance.is_valid(self.knowledge, grounder) :
                invalid_utterances.add(utterance)
        for utterance in invalid_utterances :
            utterances.remove(utterance)
        return utterances
            
    def merge_duplicate_utterances(self, utterances) :
        merged_utterances = list()
        i = 0
        while i < len(utterances) :
            u = utterances[i]
            j = i + 1
            while j < len(utterances) :
                u2 = utterances[j]
                if u2.is_like(u) :
                    #print '-------------------------'
                    #print 'Merging \n', str(u), '\n', str(u2), '\n'
                    #print '-------------------------'
                    u.parse_prob = add_log_probs(u.parse_prob, u2.parse_prob)
                    utterances.remove(u2)
                else :
                    j += 1
            merged_utterances.append(u)
            i += 1
        
        return merged_utterances    
    
    # A helper function for cluster_utterances()
    def get_utterance_type(self, utterance) :
        return utterance.action_type
        
    # A helper function for cluster_utterances()
    def get_utterance_goal(self, utterance) :
        return utterance.referring_goal
    
    # Calculate sum of log probs of utterances
    def get_sum_prob(self, utterances) :
        sum_log_prob = float('-inf')
        for utterance in utterances :
            sum_log_prob = add_log_probs(sum_log_prob, utterance.parse_prob)
        return sum_log_prob
    
    # If self.n_best_utterances has a lot of utterances that differ only
    # in one parameter, group them into a single utterance marking that
    # parameter as unknown
    # num_to_merge is the number of such utterances that need to match 
    # in this manner in order to be clustered
    def cluster_utterances(self, utterances, num_to_merge=3) :
        # Group utterances by type
        sorted_utterances = sorted(utterances, key=self.get_utterance_type)
        replacement_pairs = list()
        for utterance_type, group in itertools.groupby(sorted_utterances, self.get_utterance_type) :
            considered_utterances = list(group)
            if len(considered_utterances) >= num_to_merge :
                # If there are enough utterances of some type, group
                # them by goal
                sorted_considered_utterances = sorted(considered_utterances, key=self.get_utterance_goal)
                for goal, group in itertools.groupby(sorted_considered_utterances, self.get_utterance_goal) :
                    candidates = list(group)
                    if len(candidates) >= num_to_merge and goal is not None:
                        # If there are enough utterances of this type 
                        # having the same goal 
                        param_order = self.knowledge.param_order[goal]
                        values = dict()
                        diagreeing_params = set()
                        for utterance in candidates :
                            for param_name in param_order :
                                param_value = None
                                if param_name in utterance.referring_params :
                                    param_value = utterance.referring_params[param_name]
                                if param_value is not None :
                                    if param_name not in values :
                                        values[param_name] = set([param_value])
                                    else :
                                        values[param_name].add(param_value)
                                    if len(values[param_name]) > 1 :
                                        diagreeing_params.add(param_name)
                        #print 'diagreeing_params = ', diagreeing_params
                        if len(diagreeing_params) == 1 :
                            # There is exactly one param on whose value 
                            # these parses differ 
                            clustered_utterance = Utterance(utterance_type, goal)
                            # Create an utterance with the current type and goal
                            # Add all params values which are not the 
                            # disagreeing param        
                            params = dict()
                            for param_name in param_order :
                                if param_name not in diagreeing_params :
                                    params[param_name] = list(values[param_name])[0]
                                    # values[param_name] will have exactly
                                    # one element if param_name is not a
                                    # disagreeing param
                            clustered_utterance.referring_params = params
                            clustered_utterance.parse_prob = self.get_sum_prob(candidates)
                            replacement_pairs.append((candidates, clustered_utterance))
        for (candidates, clustered_utterance) in replacement_pairs :
            # Remove candidates from utterances and add clustered_utterance
            for utterance in candidates :
                if utterance in utterances :
                    utterances.remove(utterance)
            utterances.append(clustered_utterance)
        return utterances
            
    def get_n_best_utterances_from_parses(self, n_best_parses) :
        self.n_best_utterances = []
        N = self.parse_depth
        for [parse, conf] in n_best_parses :
            utterances = self.create_utterances_of_parse(parse)
            if utterances is not None and len(utterances) >= N :
                continue
            if utterances is not None and len(utterances) > 0:
                for utterance in utterances :
                    utterance.parse = parse
                    utterance.parse_prob = conf 
                    self.n_best_utterances.append(utterance)
        
        # Clean up by removing duplicates and invalid utterances    
        print 'Created utterances'
        print '\n'.join([str(utterance) for utterance in self.n_best_utterances])
        self.n_best_utterances = self.merge_duplicate_utterances(self.n_best_utterances)   
        print '\n\nMerged duplicates'
        print '\n'.join([str(utterance) for utterance in self.n_best_utterances])
        self.n_best_utterances = self.cluster_utterances(self.n_best_utterances)
        print '\n\nClustered to identify unknown params'
        print '\n'.join([str(utterance) for utterance in self.n_best_utterances])
        self.n_best_utterances = self.remove_invalid_utterances(self.n_best_utterances, self.grounder) 
        print '\n\nRemoved invalid utterances'
        print '\n'.join([str(utterance) for utterance in self.n_best_utterances])
        
        sum_log_prob = float('-inf')
        for utterance in self.n_best_utterances :
            sum_log_prob = add_log_probs(sum_log_prob, utterance.parse_prob)

        sum_prob = math.exp(sum_log_prob)
        #print 'sum_prob = ', sum_prob
        #print 'self.knowledge.min_obs_by_non_n_best_prob = ', self.knowledge.min_obs_by_non_n_best_prob
        non_n_best_prob = max(self.knowledge.min_obs_by_non_n_best_prob, self.knowledge.max_obs_by_non_n_best_prob - sum_prob)
        #print 'non_n_best_prob = ', non_n_best_prob
        other_utterance = Utterance('-OTHER-', parse_prob=numpy.log(non_n_best_prob))
        self.n_best_utterances.append(other_utterance)
        sum_log_prob = add_log_probs(sum_log_prob, numpy.log(non_n_best_prob))
        prob_with_utterances = list()
        
        for utterance in self.n_best_utterances :
            utterance.parse_prob -= sum_log_prob
            prob_with_utterances.append((utterance.parse_prob, utterance))
            
        prob_with_utterances.sort()
        if len(prob_with_utterances) > 0 :
            max_prob_utterance = prob_with_utterances[0][1] 
        else :
            max_prob_utterance = None
        if len(prob_with_utterances) > N :
            self.n_best_utterances = list()
            for (idx, (prob, utterance)) in enumerate(prob_with_utterances) :
                self.n_best_utterances.append(utterance)
        
        if self.max_prob_user_utterances is None :
            self.max_prob_user_utterances = list()    
        self.max_prob_user_utterances.append(max_prob_utterance)
        
        #print '\nN best utterances : '
        #print '\n'.join([str(utterance) for utterance in self.n_best_utterances])

    # Creates (text, denotation) pairs and retrains parser
    def train_parser_from_dialogue(self, final_action) :
        print 'Lexicon - ', self.parser.lexicon.surface_forms
        print 'self.parser_train_data = ', self.parser_train_data

        # Learn correct values for each param name
        answers = dict()
        goal = final_action.name
        for (idx, param_name) in enumerate(self.knowledge.param_order[goal]) :
            # Pair answers to requesting of a param to the value of the 
            # param in the final action
            answers[param_name] = final_action.params[idx]
        
        print 'answers = ', answers
        
        training_pairs = list()
        for key in self.parser_train_data :
            if key in answers :
                ccg = self.parser.lexicon.read_category_from_str('NP')
                form = self.parser.lexicon.read_semantic_form_from_str(answers[key], None, None, [],
                                                                allow_expanding_ont=False)
                form.category = ccg
                for item in self.parser_train_data[key] :
                    training_pairs.append((item, form))

        # Retrain parser
        # For everything describing the full action, get M candidate
        # parses. Try to ground each one as an action and see if it 
        # matches the desired action. If so, retrain with that parse.
        valid_semantic_forms = list()
        for command in self.parser_train_data['full'] :
            parses = self.get_n_best_parses(command, self.max_parses_examined_in_retraining) 
            for (parse, conf) in parses :
                action = self.get_action_from_parse(parse.node)
                if action is not None and action == final_action :
                    valid_semantic_forms.append(parse.node)
        
        # Heuristic - Convert the action str into a semantic node if
        # no parse was successful
        if len(self.parser_train_data['full']) > 0 and len(valid_semantic_forms) == 0 :
            ccg = self.parser.lexicon.read_category_from_str('M')
            form = self.parser.lexicon.read_semantic_form_from_str(str(final_action), None, None, [],
                                                            allow_expanding_ont=False)
            form.category = ccg
            valid_semantic_forms.append(form)
                    
        for command in self.parser_train_data['full'] :
            for parse in valid_semantic_forms :
                if self.is_parseable(command) :
                    training_pairs.append((command, parse))
                
        print 'training_pairs = \n', '\n'.join([command + ' - ' + self.parser.print_parse(parse) for (command, parse) in training_pairs])
        if len(training_pairs) >= 1 :
            self.parser.train_learner_on_semantic_forms(training_pairs)
        
        # Re-initialize the data structure for next dialogue    
        self.parser_train_data = dict()
