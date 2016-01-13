__author__ = 'aishwarya'

import sys, random, math, copy
from Knowledge import Knowledge
from HISBeliefState import HISBeliefState
from SummaryState import SummaryState
from PomdpGpSarsaPolicy import PomdpGpSarsaPolicy
from SystemAction import SystemAction
from Utterance import Utterance
from Action import Action
from TemplateBasedGenerator import TemplateBasedGenerator

class PomdpDialogAgent :

    def __init__(self, parser, grounder, u_input, output, parse_depth=10):
        self.parser = parser
        self.grounder = grounder
        self.parse_depth = parse_depth
        self.input = u_input
        self.output = output
        self.generator = TemplateBasedGenerator()

        self.knowledge = Knowledge()    
        self.state = HISBeliefState(self.knowledge)  
        self.policy = PomdpGpSarsaPolicy(self.knowledge)
        self.previous_system_action = SystemAction('repeat_goal')  
        self.n_best_utterances = None

        # define action names and functions they correspond to
        self.dialog_actions = self.knowledge.system_dialog_actions
        self.dialog_action_functions = [self.request_user_initiative, self.confirm_action, self.request_missing_param]

        self.first_turn = True
        
        # To store data for retraining the parser
        self.parser_train_data = None

    # Returns True if the dialog terminates on its own and False if the u
    # user entered stop
    def run_dialog(self) :
        self.state = HISBeliefState(self.knowledge)
        self.parser_train_data = dict()
        
        #print 'Belief state: ', str(self.state) 
        summary_state = SummaryState(self.state)
        (dialog_action, dialog_action_arg) = self.policy.get_initial_action(summary_state)    
        
        while True :
            self.state.increment_dialog_turns()
            if dialog_action == 'take_action' :
                # Submit the action given by the policy
                action = dialog_action_arg
                #self.output.say("Action: " + str(action))
                output = self.generator.get_action_sentence(action)
                self.output.say(output + " Was this the correct action?")
                response = self.input.get().lower()  
                if response == '<ERROR/>' :
                    return False  
                if response.lower() == 'y' or response.lower() == 'yes' :
                    self.policy.update_final_reward(self.knowledge.correct_action_reward)
                    self.train_parser_from_dialogue(action)
                else :
                    self.policy.update_final_reward(self.knowledge.wrong_action_reward)
                    return False
                return True
            else :
                self.previous_system_action = dialog_action_arg
                print 'System action - '
                print str(self.previous_system_action)
                # Take the dialog action
                response = self.dialog_action_functions[self.dialog_actions.index(dialog_action)]()
                if response == '<ERROR/>' :
                    return False
                if response == 'stop' :
                    self.policy.update_final_reward(self.knowledge.wrong_action_reward)
                    return False
                # Belief monitoring - update belief based on the response
                self.update_state(response)
                #print 'Belief state: ', str(self.state) 
                
                reward = self.knowledge.per_turn_reward
                summary_state = SummaryState(self.state)
                # Get the next action from the policy
                (dialog_action, dialog_action_arg) = self.policy.get_next_action(reward, summary_state)
                #print 'dialog_action = ', dialog_action
                #print str(dialog_action_arg)
            self.first_turn = False
        
    # Request the user to state/repeat their goal
    def request_user_initiative(self):
        if self.first_turn :
            self.previous_system_action.extra_data = ['first']
        output = self.generator.get_sentence(self.previous_system_action)
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
        output = self.generator.get_sentence(self.previous_system_action)
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
        #print 'response = ', response
        n_best_parses = self.parser.parse_expression(response, n=self.parse_depth)
        
        #print 'n_best_parses = '
        #for [parse, parse_tree, parse_trace, conf] in n_best_parses :
            #print 'parse = ', parse
            #print 'parse_tree = ', parse_tree
            #print 'parse_trace = ', parse_trace
            #print 'conf = ', conf 
            #print '-------------------------------'

        # Create an n-best list of Utterance objects along with probabiltiies 
        # obtained fromt ehir confidences
        self.get_n_best_utterances_from_parses(n_best_parses)
        
        if len(self.n_best_utterances) > 0 :
            # Update the belief state
            #print '\nGoing to update belief state'
            #print 'Previous system action - ', str(self.previous_system_action)
            #print 'Utterances - ', '\n'.join([str(utterance) for utterance in self.n_best_utterances])
            self.state.update(self.previous_system_action, self.n_best_utterances, self.grounder)

    # Confirm a (possibly partial) action
    def confirm_action(self):
        # with reverse parsing, want to confirm(a.name(a.params))
        # for now, just use a.name and a.params raw
        #system_action = self.previous_system_action
        #params = []
        #goal = system_action.referring_goal
        #if system_action.referring_params is not None :
            #param_order = self.knowledge.param_order[goal]
            #for param_name in param_order :
                #if param_name in system_action.referring_params :
                    #params.append(system_action.referring_params[param_name])
        #if goal is None :
            #self.output.say("I should take action involving " +
                        #','.join([str(p) for p in params if p is not None]) + "?")
        #else :
            #self.output.say("I should take action " + goal +" involving " +
                        #','.join([str(p) for p in params if p is not None]) + "?")
        output = self.generator.get_sentence(self.previous_system_action)
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
                utterance = Utterance('inform', goal)
            else :
                params = dict()
                for (idx, param_val) in enumerate(action.params) :
                    param_name = self.knowledge.param_order[goal][idx]
                    if param_val != 'UNK_E' :
                        params[param_name] = param_val
                utterance = Utterance('inform', goal, params)
        return utterance

    def create_utterances_of_parse(self, parse) :
        #print 'Creating utterances of parse '
        #print 'parse = ', parse
        #print 'type(parse) = ', type(parse)
        #print 'self.parser.print_parse(parse) = ', self.parser.print_parse(parse)
        #print 'lexical form = ', self.parser.print_parse(self.parser.lexicon.semantic_forms[54])
        #print 'Parser ontology: ', self.parser.ontology.preds
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
            utterance = Utterance('inform', goal, params)      
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
                if self.previous_system_action.extra_data is not None and len(self.previous_system_action.extra_data) > 1:
                    param_name = self.previous_system_action.extra_data[0]
                else :
                    param_name = 'patient'
                goal = self.previous_system_action.referring_goal
                params = dict()
                params[param_name] = answer
                utterance = Utterance('inform', goal, params)      
                utterances.append(utterance)
            return utterances
        except TypeError as e :
            print e.message
            
    def merge_duplicate_utterances(self, utterances) :
        n = len(utterances)
        merged_utterances = list()
        
        i = 0
        while i < n :
            #print 'n = ', n, ', i = ', i
            u = utterances[i]
            n = len(utterances)
            j = i
            while j < n :
                #print 'n = ', n, ', j = ', j
                u2 = utterances[j]
                if u2.is_like(u) :
                    u.parse_prob += u2.parse_prob
                    utterances.remove(u2)
                    n -= 1
                else :
                    j += 1
                #print 'n = ', n, ', j = ', j
                #print
            merged_utterances.append(u)
            i += 1
        
        return merged_utterances    
            
    def get_n_best_utterances_from_parses(self, n_best_parses) :
        #print "In get_n_best_utterances_from_parses"
        #print "No of parses = ", len(n_best_parses)
        sum_exp_conf = 0.0
        self.n_best_utterances = []
        N = self.parser.beam_width
        for [parse, parse_tree, parse_trace, conf] in n_best_parses :
            #print "\nParsing ", self.parser.print_parse(parse)
            #print 'conf = ', conf
            #print '-------------------------------'
            #print 'parse = ', parse
            #print 'parse_tree = ', parse_tree
            #print 'parse_trace = ', parse_trace
            #print 'conf = ', conf 

            utterances = self.create_utterances_of_parse(parse)
            if utterances is not None and len(utterances) >= N :
                continue
            #print "Returned"
            if utterances is not None and len(utterances) > 0:
                #print "Got ", len(utterances), " utterances"
                for utterance in utterances :
                    #print str(utterance)
                    utterance.parse_prob = math.exp(conf) / len(utterances)
                    self.n_best_utterances.append(utterance)
                    #sum_exp_conf += utterance.parse_prob
            
        self.n_best_utterances = self.merge_duplicate_utterances(self.n_best_utterances)    
        
        sum_exp_conf = 0
        for utterance in self.n_best_utterances :
            sum_exp_conf += utterance.parse_prob
            
        sum_exp_conf += self.knowledge.obs_by_non_n_best_prob
        for utterance in self.n_best_utterances :
            utterance.parse_prob /= sum_exp_conf
            
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
        print 'Lexicon - ', self.parser.lexicon.surface_forms
        
        print 'self.parser_train_data = ', self.parser_train_data
        
        answers = dict()
        answers['full'] = str(final_action)
        goal = final_action.name
        for (idx, param_name) in enumerate(self.knowledge.param_order[goal]) :
            answers[param_name] = final_action.params[idx]
        
        print 'answers = ', answers
        
        new_lexical_entries = list()
        
        for key in self.parser_train_data :
            if key != 'full' :
                # This is a param value. It may be a multi-word expression
                # present in the lexicon
                for value in self.parser_train_data[key] :
                    if len(self.parser.lexicon.get_semantic_forms_for_surface_form(value)) > 0 :
                        # Known expression. No need for new lexical entry
                        continue
                    else :
                        # If the first word is a, an, the, see if the rest
                        # of it is present in the lexicon
                        tokens = self.parser.tokenize(value)
                        if tokens[0] == 'a' or tokens[0] == 'an' or tokens[0] == 'the' :
                            rest_of_it = ' '.join(tokens[1:])
                            if len(self.parser.lexicon.get_semantic_forms_for_surface_form(rest_of_it)) > 0 :
                                continue
                            else :
                                del tokens[0]
                        
                        # If it is of the form x's office and x is known 
                        # again it can be ignored
                        if tokens[-1] == 'office' :
                            rest_of_it = ' '.join(tokens[:-2])
                            if len(self.parser.lexicon.get_semantic_forms_for_surface_form(rest_of_it)) > 0 :
                                continue
                            # Else you'd ideally want to use the knowledge 
                            # base to find who's office is the room you're 
                            # talking about and hence create an entry for x
                        
                        # If it has too many words it is unlikely to be 
                        # a multi-word expression to be learnt as such
                        if len(tokens) > 3 :
                            continue
                    
                        surface_form = ' '.join(tokens)
                        if key in answers :
                            semantic_form = answers[key]
                            entry = surface_form + ' :- N : ' + semantic_form
                            print 'Creating entry ', entry 
                            new_lexical_entries.append(entry)
            
            else :    
                for value in self.parser_train_data[key] :            
                    tokens = self.parser.tokenize(value)
                    unknown_surface_forms = [(idx, token) for (idx, token) in enumerate(tokens) if len(self.parser.lexicon.get_semantic_forms_for_surface_form(token)) == 0]
                    non_multi_word_expressions = list()
                    possible_matchers = tokens 
                    for (idx, token) in unknown_surface_forms :
                        # Bad heuristic to check for multi-word expressions
                        possible_multi_word_tokens = []
                        if idx >= 1 :
                            possible_multi_word_tokens.append(tokens[idx-1:idx+1])
                        if idx >= 2 :
                            possible_multi_word_tokens.append(tokens[idx-2:idx+1])
                        if idx >= 3 :
                            possible_multi_word_tokens.append(tokens[idx-3:idx+1])
                        if idx <= len(tokens) - 2 :
                            possible_multi_word_tokens.append(tokens[idx:idx+1])
                        if idx <= len(tokens) - 3 :
                            possible_multi_word_tokens.append(tokens[idx:idx+2])
                        if idx <= len(tokens) - 4 :
                            possible_multi_word_tokens.append(tokens[idx:idx+3])
                        match_found = False
                        for multi_word_token in possible_multi_word_tokens :
                            text = ' '.join(multi_word_token)
                            if len(self.parser.lexicon.get_semantic_forms_for_surface_form(text)) > 0 :
                                possible_matchers.append(text)
                                match_found = True
                                break
                        if not match_found :
                            non_multi_word_expressions.append(token)
                    
                    if len(non_multi_word_expressions) == 1 :
                        # Try to create lexical entries only if exactly one
                        # unknown surface form
                        unknown_surface_form = non_multi_word_expressions[0]
                        possible_matches = final_action.params
                        if unknown_surface_form + '\'s office' not in value :
                            # We don't really know how to do the whole office
                            # thing yet
                            for matcher in possible_matchers :
                                semantic_indices = self.parser.lexicon.get_semantic_forms_for_surface_form(matcher)
                                for idx in semantic_indices :
                                    semantic_form = self.parser.lexicon.semantic_forms[idx]
                                    if semantic_form.idx is not None :
                                        ont_value = self.parser.lexicon.ontology.preds[semantic_form.idx]
                                        if ont_value in possible_matches :
                                            possible_matches.remove(ont_value)
                            print 'possible_matches = ', possible_matches
                            if len(possible_matches) == 1 :
                                # Only add an extry if it appears unambiguous
                                entry = unknown_surface_form + ' :- N : ' + possible_matches[0]
                                print 'Creating entry from sentence ', entry
                                new_lexical_entries.append(entry)
                             
        print 'new_lexical_entries = ', new_lexical_entries                
        print 'Retraining parser'
        
        self.parser.lexicon.expand_lex_from_strs(
                    new_lexical_entries, self.parser.lexicon.surface_forms, self.parser.lexicon.semantic_forms, 
                    self.parser.lexicon.entries, self.parser.lexicon.pred_to_surface, 
                    allow_expanding_ont=False)
                    
        training_pairs = list()
        for key in self.parser_train_data :
            if key in answers :
                for item in self.parser_train_data[key] :
                    training_pairs.append((item, answers[key]))

        self.parser.train_learner_on_denotations(training_pairs, 10, 4, 4)

        print 'Finished retraining parser'

        self.parser_train_data = dict()
