__author__ = 'aishwarya'

import sys, random, math, copy, ast
from os import listdir
from os.path import isfile, join
import numpy as np

from Knowledge import Knowledge
from HISBeliefState import HISBeliefState
from SummaryState import SummaryState
from PomdpGpSarsaPolicy import PomdpGpSarsaPolicy
from PomdpKtdqPolicy import PomdpKtdqPolicy
from SystemAction import SystemAction
from Utterance import Utterance
from Action import Action
from TemplateBasedGenerator import TemplateBasedGenerator
from Utils import *

# This class is supposed to read log files and convert them into a form 
# that can be used for training a policy. It requires the ability to 
# perform the NL understanding and belief monitoring.
class PomdpTrainer :

    def __init__(self, parser, grounder, parse_depth=10, load_policy_from_file=False):
        self.parser = parser
        self.grounder = grounder
        self.parse_depth = parse_depth
        self.generator = TemplateBasedGenerator()

        self.knowledge = Knowledge()    
        self.state = HISBeliefState(self.knowledge)  
        self.policy = PomdpKtdqPolicy(self.knowledge, load_policy_from_file)
        self.previous_system_action = SystemAction('repeat_goal')  
        self.n_best_utterances = None

        self.first_turn = True
        
        # To store data for retraining the parser
        self.parser_train_data = None
        self.max_prob_user_utterances = None

    def train_from_old_logs(self, success_dir, fail_dir) :
        success_files = [f for f in listdir(success_dir) if isfile(join(success_dir, f))]
        fail_files = [f for f in listdir(fail_dir) if isfile(join(fail_dir, f))]
        success_idx = 0
        fail_idx = 0
        while success_idx < len(success_files) or fail_idx < len(fail_files) :
            r = np.random.uniform()
            if r < 0.5 :
                fname = success_dir + '/' + success_files[success_idx]
                self.train_policy_from_single_old_log(fname, True)
                success_idx += 1
            else :
                fname = fail_dir + '/' + fail_files[fail_idx]
                self.train_policy_from_single_old_log(fname, False)
                fail_idx += 1
            

    # Train the policy using logs from the IJCAI 2015 experiment
    # This makes assumptions on the formatting of the logs
    def train_policy_from_single_old_log(self, conv_log_name, dialogue_successful=True) :
        conv_log = open(conv_log_name, 'r')
        self.max_prob_user_utterances = list()
        self.parser_train_data = dict()
        for line in conv_log :
            parts = line.split('\t')
            print '---------------------------------------'
            print 'parts = ', parts
            print '---------------------------------------'
            if parts[0] == 'USER' :
                if self.previous_system_action is not None :
                    prev_state = SummaryState(self.state)
                    response = parts[1]
                    print 'Updating state for response :', response
                    self.update_state(response)
                    next_state = SummaryState(self.state)
                    print '---------------------------------------'
                    print '---------------------------------------'
                    print 'prev_state : ', str(prev_state)
                    print '---------------------------------------'
                    print 'action : ', str(self.previous_system_action)
                    print '---------------------------------------'
                    print 'next_state : ', str(next_state)
                    print '---------------------------------------'
                    self.policy.train(prev_state, self.previous_system_action.action_type, next_state, self.knowledge.per_turn_reward)
                    self.previous_system_action = None
            else :
                if len(parts) != 6 :
                    continue
                # Create system action and set as prev system action
                if parts[4] == 'user_initiative' :
                    self.previous_system_action = SystemAction('repeat_goal')
                elif parts[4] == 'system_initiative' :
                    # Disambiguate between confirm and request missing param
                    details = ast.literal_eval(parts[3])
                    goal = None
                    params = dict()
                    if details[0] == 'at' :
                        goal = 'at'
                        if details[2] is not None :
                            params['location'] = details[2].strip().split(':')[0]
                    elif details[0] == 'served' :
                        goal = 'bring'
                        if details[1] is not None :
                            params['patient'] = details[1].strip().split(':')[0]
                        if details[2] is not None :
                            params['recipient'] = details[2].strip().split(':')[0]
                    target = parts[5].strip()
                    details_header = ['action', 'patient', 'recipient']
                    if target in details_header :
                        if details[details_header.index(target)] is None :
                            action_type = 'request_missing_param'
                        else :
                            action_type = 'confirm_action'
                        self.previous_system_action = SystemAction(action_type, goal, params)
                    else :
                        self.previous_system_action = None
                else :
                    # Not useful dialogue steps. Mostly robot politeness
                    self.previous_system_action = None
                print '---------------------------------------'
                print 'action : ', str(self.previous_system_action)
                print '---------------------------------------'
        
        # Need to feed it with terminal reward
        prev_state = SummaryState(self.state)
        if dialogue_successful :
            self.policy.train(prev_state, 'take_action', None, self.knowledge.correct_action_reward)
        else :
            self.policy.train(prev_state, 'take_action', None, self.knowledge.wrong_action_reward)
    
    def get_n_best_parses(self, response, n=None) :
        if n is None :
            n = self.parse_depth
        # Parser expects a space between 's and the thing it is applied 
        # to, for example "alice 's" rather than "alice's"
        response = re.sub("'s", " 's", response)
        parse_generator = self.parser.most_likely_cky_parse(response)
        k = 0
        parses = list()
        for (parse, score, _) in parse_generator :
            print 'parse = ', self.parser.print_parse(parse.node, show_category=True), ', score = ', score 
            parses.append((parse, score))
            k += 1
            if k == n :
                break
        return parses
             
    # Parse the user response and update belief state
    def update_state(self, response) :
        # get n best parses for confirmation
        #print 'response = ', response
        n_best_parses = self.parser.parse_expression(response, n=self.parse_depth)
        print 'Parsed'
        #print 'n_best_parses = '
        #for [parse, parse_tree, parse_trace, conf] in n_best_parses :
            #print 'parse = ', self.parser.print_parse(parse)
            #print 'parse_tree = ', self.parser.print_parse(parse_tree[0])
            #print 'parse_trace[-1] = ', parse_trace[-1]
            #print 'parse_trace = ', parse_trace
            #print 'type(parse_trace) = ', type(parse_trace)
            #print 'len(parse_trace) = ', len(parse_trace)
            #print 'conf = ', conf 
            #print '-------------------------------'

        # Create an n-best list of Utterance objects along with probabiltiies 
        # obtained from their confidences
        self.get_n_best_utterances_from_parses(n_best_parses)
        print 'Created utterances'
        
        if len(self.n_best_utterances) > 0 :
            # Update the belief state
            print '\nGoing to update belief state'
            print 'Previous system action - ', str(self.previous_system_action)
            #print 'Utterances - ', '\n'.join([str(utterance) for utterance in self.n_best_utterances])
            print 'Utterances - '
            for utterance in self.n_best_utterances :
                print str(utterance)
                print '\tParse = ', self.parser.print_parse(utterance.parse)
            self.state.update(self.previous_system_action, self.n_best_utterances, self.grounder)
            print 'Updated'

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
        # First check if it is a confirmation or denial
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
        
        #print 'Neither affirm nor deny'
        
        # Now check if it a full inform - mentions goal and params
        try:
            #print 'Trying to get an action'
            p_action = self.get_action_from_parse(parse)
        except SystemError:
            p_action = None

        if p_action is not None :
            #print 'Got an action. Will convert and return'
            return [self.convert_action_to_utterance(p_action)]

        #print 'Did not get a fully specified action' 

        # if this failed, try again allowing missing lambdas to become UNK without token ties
        # print "Going to retry parsing"
        
        if parse.idx == self.parser.ontology.preds.index('speak_e') or parse.idx == self.parser.ontology.preds.index('speak_t'):
            # Don't want UNK parses for speak actions
            return []
        
        if parse.idx != None and self.parser.ontology.preds[parse.idx] in ['speak_e', 'speak_t'] :
            # Don't want UNK parses for speak actions
            return []
        if parse.children is not None and len(parse.children) >= 1 :
            if parse.children[0].idx is not None and self.parser.ontology.preds[parse.children[0].idx] in ['speak_e', 'speak_t'] :
                # Don't want UNK parses for speak actions
                return []

        #print 'Going to try creating an UNK parse'

        UNK_root = copy.deepcopy(parse)
        curr = UNK_root
        heading_lambdas = []
        if curr.is_lambda and curr.is_lambda_instantiation:
            heading_lambdas.append(curr.lambda_name)
            curr = curr.children[0]
        if curr.children is not None:
            for i in range(0, len(curr.children)):
                if curr.children[i].is_lambda and curr.children[i].lambda_name in heading_lambdas:
                    curr.children[i] = self.parser.create_unk_node()
        try:
            #print 'Trying to get UNK action'
            p_unk_action = self.get_action_from_parse(UNK_root)
        except SystemError:
            p_unk_action = None
        except TypeError as e:
            return []
            print '--------------------------------'
            print e.message
            print '--------------------------------'
            print self.parser.print_parse(parse)
            print '--------------------------------'
            print 'parse.idx = ', parse.idx
            if parse.idx != None :
                print 'self.parser.ontology.preds[parse.idx] = ', self.parser.ontology.preds[parse.idx]
            if parse.children is not None and len(parse.children) >= 1 :
                print 'parse.children[0].idx = ', parse.children[0].idx
                if parse.children[0].idx is not None :
                    print 'self.parser.ontology.preds[parse.children[0].idx] = ', self.parser.ontology.preds[parse.children[0].idx]
            #print 'parse.children = '
            #for c in parse.children :
                #print self.parser.print_parse(c)
                #print '\n'
            print '--------------------------------'
            
        if p_unk_action is not None :
            #print 'Got UNK action. Going to convert and return'
            return [self.convert_action_to_utterance(p_unk_action)]
            
        # Getting a complete action failed so assume this is a param  
        #print 'Trying to ground as param value'
        #print "\nGrounding ", self.parser.print_parse(parse)
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
            print e.message

    def remove_invalid_utterances(self, utterances, grounder) :
        invalid_utterances = set()
        for utterance in utterances :
            if type(utterance) == 'Utterance' and not utterance.is_valid(self.knowledge, grounder) :
                invalid_utterances.add(utterance)
        for utterance in invalid_utterances :
            utterances.remove(utterance)
        return utterances
            
    def merge_duplicate_utterances(self, utterances) :
        merged_utterances = list()
        i = 0
        while i < len(utterances) :
            u = utterances[i]
            j = i
            while j < len(utterances) :
                u2 = utterances[j]
                if u2.is_like(u) :
                    u.parse_prob = add_log_probs(u.parse_prob, u2.parse_prob)
                    utterances.remove(u2)
                else :
                    j += 1
            merged_utterances.append(u)
            i += 1
        
        return merged_utterances    
              
    def get_n_best_utterances_from_parses(self, n_best_parses) :
        #print "In get_n_best_utterances_from_parses"
        #print "No of parses = ", len(n_best_parses)
        self.n_best_utterances = []
        N = self.parse_depth
        for [parse, conf] in n_best_parses :
            utterances = self.create_utterances_of_parse(parse)
            if utterances is not None and len(utterances) >= N :
                continue
            #print "Returned"
            if utterances is not None and len(utterances) > 0:
                #print "Got ", len(utterances), " utterances"
                for utterance in utterances :
                    utterance.parse = parse
                    #print str(utterance)
                    #utterance.parse_prob = math.exp(conf) / len(utterances)
                    utterance.parse_prob = conf 
                    self.n_best_utterances.append(utterance)
        
        # Clean up by removing duplicates and invalid utterances    
        self.n_best_utterances = self.merge_duplicate_utterances(self.n_best_utterances)   
        self.n_best_utterances = self.remove_invalid_utterances(self.n_best_utterances, self.grounder) 
        
        sum_exp_conf = float('-inf')
        for utterance in self.n_best_utterances :
            sum_exp_conf = add_log_probs(sum_exp_conf, utterance.parse_prob)
        
        sum_exp_conf = add_log_probs(sum_exp_conf, np.log(self.knowledge.obs_by_non_n_best_prob))
        prob_with_utterances = list()
        
        for utterance in self.n_best_utterances :
            utterance.parse_prob -= sum_exp_conf
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
            
        self.max_prob_user_utterances.append(max_prob_utterance)
        #print "N-best utterances: "
        #for utterance in self.n_best_utterances :
            #print str(utterance)

    def read_in_utterance_action_pairs(self, fname):
        f = open(fname, 'r')
        f_lines = f.readlines()
        pairs = []
        for i in range(0,len(f_lines), 4):
            t = self.parser.tokenize(f_lines[i].strip())
            cat, r = self.parser.lexicon.read_syn_sem(f_lines[i+1].strip())
            a_str = f_lines[i+2].strip()
            a_name, a_params_str = a_str.strip(')').split('(')
            a_params = a_params_str.split(',')
            for j in range(0, len(a_params)):
                if a_params[j] == "True":
                    a_params[j] = True
                elif a_params[j] == "False":
                    a_params[j] = False
            pairs.append([t, r, Action(a_name, a_params)])
        f.close()
        return pairs

    def train_parser_from_utterance_action_pairs(self, pairs, epochs=10, parse_beam=10):
        for e in range(0, epochs):
            print "training epoch "+str(e)
            random.shuffle(pairs)
            train_data = []
            num_correct = 0
            for t, r, a in pairs:
                n_best_parses = self.parser.parse_tokens(t, n=parse_beam)
                if len(n_best_parses) == 0:
                    print "WARNING: no parses found for tokens "+str(t)
                    continue
                a_chosen = None
                a_candidate = None
                correct_found = False
                for i in range(0, len(n_best_parses)):
                    try:
                        # print "parse: " + self.parser.print_parse(n_best_parses[i][0])  # DEBUG
                        # print self.parser.print_semantic_parse_result(n_best_parses[i][1])  # DEBUG
                        a_candidate = self.get_action_from_parse(n_best_parses[i][0])
                        # print "candidate: "+str(a_candidate)  # DEBUG
                    except SystemError:
                        a_candidate = Action()
                    if i == 0:
                        a_chosen = a_candidate
                    if a_candidate.__eq__(a):
                        correct_found = True
                        self.parser.add_genlex_entries_to_lexicon_from_partial(n_best_parses[i][1])
                        break  # the action matches gold and this parse should be used in training
                if not correct_found:
                    print "WARNING: could not find correct action '"+str(a)+"' for tokens "+str(t)
                if a_chosen != a_candidate:
                    train_data.append([t, n_best_parses[0], a_chosen, t, n_best_parses[i], a])
                else:
                    num_correct += 1
            print "\t"+str(num_correct)+"/"+str(len(pairs))+" top choices"
            if num_correct == len(pairs):
                print "WARNING: training converged at epoch "+str(e)+"/"+str(epochs)
                return True
            self.parser.learner.learn_from_actions(train_data)
        return False

    def get_action_from_parse(self, root):
	# print "Inside get_action_from_parse"

        # print "parse to get action from: " + self.parser.print_parse(root)  # DEBUG

        root.set_return_type(self.parser.ontology)  # in case this was not calculated during parsing

        # execute action from imperative utterance
        if root.return_type == self.parser.ontology.types.index('a'):
            # assume for now that logical connectives do not operate over actions (eg. no (do action a and action b))
            # ground action arguments
            action = self.parser.ontology.preds[root.idx]
            # print "action: "+action  # DEBUG
            g_args = []
            # print "Going to enter loop"
            for arg in root.children:
                # print "In loop"
                #print "Grounding ", self.parser.print_parse(arg)
                g = self.grounder.groundSemanticNode(arg, [], [], [])
                # print "Grounded"
                # print "arg: "+str(g)  # DEBUG
                answer = self.grounder.grounding_to_answer_set(g)
                #print 'answers = ', answer
                #print '--------------------------------'
                # print "Interpreted grounding"
                if len(answer) == 0:
                    # print "Single answer found"
                    if action == "speak_t":
                        g_args.append(False)
                    elif action == "speak_e":
                        g_args.append(None)
                    else:
                        raise SystemError("action argument unrecognized. arg: '"+str(answer)+"'")
                elif len(answer) > 1:
                    raise SystemError("multiple interpretations of action argument renders command ambiguous. arg: '"+str(answer)+"'")
                else:
                    # print "No answer found"
                    if action == "speak_t":
                        g_args.append(True)
                    else:
                        g_args.append(answer[0])
            # print "args: "+str(g_args)  # DEBUG
            return Action(action, g_args)

        else:
            raise SystemError("cannot get action from return type "+str(self.parser.ontology.types[root.return_type]))

    def train_parser_from_dialogue(self, final_action) :
        #print 'Lexicon - ', self.parser.lexicon.surface_forms
        
        #print 'self.parser_train_data = ', self.parser_train_data
        
        #answers = dict()
        #answers['full'] = str(final_action)
        #goal = final_action.name
        #for (idx, param_name) in enumerate(self.knowledge.param_order[goal]) :
            #answers[param_name] = final_action.params[idx]
        
        #print 'answers = ', answers
        
        #new_lexical_entries = list()
        
        #for key in self.parser_train_data :
            #if key != 'full' :
                ## This is a param value. It may be a multi-word expression
                ## present in the lexicon
                #for value in self.parser_train_data[key] :
                    #if len(self.parser.lexicon.get_semantic_forms_for_surface_form(value)) > 0 :
                        ## Known expression. No need for new lexical entry
                        #continue
                    #else :
                        ## If the first word is a, an, the, see if the rest
                        ## of it is present in the lexicon
                        #tokens = self.parser.tokenize(value)
                        #if tokens[0] == 'a' or tokens[0] == 'an' or tokens[0] == 'the' :
                            #rest_of_it = ' '.join(tokens[1:])
                            #if len(self.parser.lexicon.get_semantic_forms_for_surface_form(rest_of_it)) > 0 :
                                #continue
                            #else :
                                #del tokens[0]
                        
                        ## If it is of the form x's office and x is known 
                        ## again it can be ignored
                        #if tokens[-1] == 'office' :
                            #rest_of_it = ' '.join(tokens[:-2])
                            #if len(self.parser.lexicon.get_semantic_forms_for_surface_form(rest_of_it)) > 0 :
                                #continue
                            ## Else you'd ideally want to use the knowledge 
                            ## base to find who's office is the room you're 
                            ## talking about and hence create an entry for x
                        
                        ## If it has too many words it is unlikely to be 
                        ## a multi-word expression to be learnt as such
                        #if len(tokens) > 3 :
                            #continue
                    
                        #surface_form = ' '.join(tokens)
                        #if key in answers :
                            #semantic_form = answers[key]
                            #entry = surface_form + ' :- N : ' + semantic_form
                            #print 'Creating entry ', entry 
                            #new_lexical_entries.append(entry)
            
            #else :    
                #for value in self.parser_train_data[key] :            
                    #tokens = self.parser.tokenize(value)
                    #unknown_surface_forms = [(idx, token) for (idx, token) in enumerate(tokens) if len(self.parser.lexicon.get_semantic_forms_for_surface_form(token)) == 0]
                    #non_multi_word_expressions = list()
                    #possible_matchers = tokens 
                    #for (idx, token) in unknown_surface_forms :
                        ## Bad heuristic to check for multi-word expressions
                        #possible_multi_word_tokens = []
                        #if idx >= 1 :
                            #possible_multi_word_tokens.append(tokens[idx-1:idx+1])
                        #if idx >= 2 :
                            #possible_multi_word_tokens.append(tokens[idx-2:idx+1])
                        #if idx >= 3 :
                            #possible_multi_word_tokens.append(tokens[idx-3:idx+1])
                        #if idx <= len(tokens) - 2 :
                            #possible_multi_word_tokens.append(tokens[idx:idx+1])
                        #if idx <= len(tokens) - 3 :
                            #possible_multi_word_tokens.append(tokens[idx:idx+2])
                        #if idx <= len(tokens) - 4 :
                            #possible_multi_word_tokens.append(tokens[idx:idx+3])
                        #match_found = False
                        #for multi_word_token in possible_multi_word_tokens :
                            #text = ' '.join(multi_word_token)
                            #if len(self.parser.lexicon.get_semantic_forms_for_surface_form(text)) > 0 :
                                #possible_matchers.append(text)
                                #match_found = True
                                #break
                        #if not match_found :
                            #non_multi_word_expressions.append(token)
                    
                    #if len(non_multi_word_expressions) == 1 :
                        ## Try to create lexical entries only if exactly one
                        ## unknown surface form
                        #unknown_surface_form = non_multi_word_expressions[0]
                        #possible_matches = final_action.params
                        #if unknown_surface_form + '\'s office' not in value :
                            ## We don't really know how to do the whole office
                            ## thing yet
                            #for matcher in possible_matchers :
                                #semantic_indices = self.parser.lexicon.get_semantic_forms_for_surface_form(matcher)
                                #for idx in semantic_indices :
                                    #semantic_form = self.parser.lexicon.semantic_forms[idx]
                                    #if semantic_form.idx is not None :
                                        #ont_value = self.parser.lexicon.ontology.preds[semantic_form.idx]
                                        #if ont_value in possible_matches :
                                            #possible_matches.remove(ont_value)
                            #print 'possible_matches = ', possible_matches
                            #if len(possible_matches) == 1 :
                                ## Only add an extry if it appears unambiguous
                                #entry = unknown_surface_form + ' :- N : ' + possible_matches[0]
                                #print 'Creating entry from sentence ', entry
                                #new_lexical_entries.append(entry)
                             
        #print 'new_lexical_entries = ', new_lexical_entries                
        ##print 'Retraining parser'
        
        ## Add entries to lexicon
        #self.parser.lexicon.expand_lex_from_strs(
                    #new_lexical_entries, self.parser.lexicon.surface_forms, self.parser.lexicon.semantic_forms, 
                    #self.parser.lexicon.entries, self.parser.lexicon.pred_to_surface, 
                    #allow_expanding_ont=False)
                    
        ## Log the added lexicon entries   
        #if self.lexical_addition_log is not None :
            #f = open(self.lexical_addition_log, 'a')
            #for entry in new_lexical_entries :
                #f.write(entry + '\n')
            #f.close()
                    
        #training_pairs = list()
        #for key in self.parser_train_data :
            #if key in answers :
                #for item in self.parser_train_data[key] :
                    #training_pairs.append((item, answers[key]))

        #print 'training_pairs = ', training_pairs

        #self.parser.train_learner_on_denotations(training_pairs, 10, 100, 3)

        #print 'Finished retraining parser'
        
        #print 'Looking for None parses'
        
        #for utterance in self.max_prob_user_utterances :
            #print 'Examining ', utterance.parse_leaves
            #words_matched_to_none = list()
            #print 'words_matched_to_none = ', words_matched_to_none
            #for (idx, (ont_pred, word)) in enumerate(utterance.parse_leaves[1]) :
                #if ont_pred is None :
                    ## In the most confident parse, this word was ignored
                    #if len(self.parser.lexicon.get_semantic_forms_for_surface_form(word)) == 0 :
                        ## There is no entry for this word in the lexicon
                        #words_matched_to_none.append((idx, ont_pred, word))
                        
            ## Avoid dealing with ambiguity
            #if len(words_matched_to_none) == 1 :
                ## See whether the thing on either side is a noun
                #(idx, ont_pred, word) = words_matched_to_none[0]
                #print 'Going to try for (', idx, ', ', ont_pred, ', ', word, ')'
                #lexicon_entry = None
                #if idx > 0 :
                    #sem_indices = self.parser.lexicon.get_semantic_forms_for_surface_form(utterance.parse_leaves[1][idx-1][1])
                    #sem_forms = [self.parser.lexicon.semantic_forms[sem_idx] for sem_idx in sem_indices]
                    #if len(sem_forms) == 1 :
                        #if sem_forms[0].category == self.parser.lexicon.read_category_from_str('N') :
                            #ont_idx = sem_forms[0].idx
                            #ont_pred = self.parser.ontology.preds[ont_idx]
                            ## Word before is a noun
                            #lexicon_entry = word + ' :- N : ' + ont_pred
                #if lexicon_entry is None and idx < len(utterance.parse_leaves[1]) - 1 :
                    #sem_forms = self.parser.lexicon.get_semantic_forms_for_surface_form(utterance.parse_leaves[1][idx+1][1])
                    #if len(sem_forms) == 1 :
                        #if sem_forms[0].category == self.parser.lexicon.read_category_from_str('N') :
                            #ont_idx = sem_forms[0].idx
                            #ont_pred = self.parser.ontology.preds[ont_idx]
                            ## Word before is a noun
                            #lexicon_entry = word + ' :- N : ' + ont_pred
                
                #print 'lexicon_entry = ', lexicon_entry
                ## If one of them is, combine with that - add this entry to lexicon
                #if lexicon_entry is not None :
                    #self.parser.lexicon.expand_lex_from_strs([lexicon_entry], 
                        #self.parser.lexicon.surface_forms, self.parser.lexicon.semantic_forms, 
                        #self.parser.lexicon.entries, self.parser.lexicon.pred_to_surface, 
                        #allow_expanding_ont=False)
                
                ## Re-parse expression
                #print 'Parsing expression'
                #expr = ' '.join([w for (o, w) in utterance.parse_leaves[1]])
                #n_best_parses = self.parser.parse_expression(' '.join(expr), k=100, n=1)
                
                ## Create utterances of parse
                #print 'Creating utterances'
                #self.get_n_best_utterances_from_parses(n_best_parses)
                    ## Writes n best utterances into self.n_best_utterances
                
                ## Check that new max prob utterance matches old max prob utterance = utterance
                #new_max_prob_utterance = None
                #for new_utterance in self.n_best_utterances :
                    #if new_max_prob_utterance is None or new_utterance.parse_prob > new_max_prob_utterance.parse_prob :
                        #new_max_prob_utterance = new_utterance
                
                ## If not, delete the lexical entry just added
                #if not utterance.is_like(new_max_prob_utterance) and ont_idx in self.parser.ontology.preds :
                    #print 'Utterances not matching. Deleting entry.'
                    #self.parser.lexicon.delete_semantic_form_for_surface_form(word, self.parser.ontology.preds.index(ont_idx))

        self.parser_train_data = dict()
        self.max_prob_user_utterances = list()
        save_model(self.parser, 'pomdp_parser')
        
        
