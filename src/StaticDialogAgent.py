__author__ = 'aishwarya'

import random
import copy
import sys
import Action
import StaticDialogState
from TemplateBasedGenerator import TemplateBasedGenerator
from SystemAction import SystemAction
from Knowledge import Knowledge
from Utils import *
from StaticDialogStateWithTypechecking import StaticDialogStateWithTypechecking

class StaticDialogAgent:

    def __init__(self, parser, generator, grounder, policy, u_input, output, parse_depth=10):
        self.parser = parser
        self.generator = generator
        self.grounder = grounder
        self.parse_depth = parse_depth
        self.state = None
        self.policy = policy
        self.input = u_input
        self.output = output

        # define action names and functions they correspond to
        self.dialog_actions = ["repeat_goal", "confirm_action", "request_missing_param"]
        self.dialog_action_functions = [self.request_user_initiative, self.confirm_action, self.request_missing_param]

        self.response_generator = TemplateBasedGenerator()
        self.knowledge = Knowledge()

    # initiate a new dialog with the agent with initial utterance u
    def initiate_dialog_to_get_action(self, u):
        self.parser_train_data = dict()
        #print "Function call succeeded"         

        #self.state = StaticDialogState.StaticDialogState()
        self.state = StaticDialogStateWithTypechecking()
        #print "Created state"
        self.update_state_from_user_initiative(u)
        #print "Updated state"

        # select next action from state
        action = None
        while action is None:
            print "dialog state: "+str(self.state)  # DEBUG
            action = self.policy.resolve_state_to_action(self.state)

            # if the state does not clearly define a user goal, take a dialog action to move towards it
            if action is None:
                dialog_action, dialog_action_args = self.policy.select_dialog_action(self.state)
                # print "Going to take action: ", str(dialog_action)
                if dialog_action not in self.dialog_actions:
                    sys.exit("ERROR: unrecognized dialog action '"+dialog_action+"' returned by policy for state "+str(self.state))
                self.state.previous_action = [dialog_action, dialog_action_args]
                self.dialog_action_functions[self.dialog_actions.index(dialog_action)](dialog_action_args)

        self.train_parser_from_dialogue(action)

        return action

    # request the user repeat their original goal
    def request_user_initiative(self, args):
        #self.output.say("Can you restate your question or command?")
        self.output.say("I\'m sorry but could you clarify again what you wanted me to do?")
        u = self.input.get()
        if 'full' in self.parser_train_data :
            self.parser_train_data['full'].append(u)
        else :
            self.parser_train_data['full'] = [u]
        self.update_state_from_user_initiative(u)

    # request a missing action parameter
    def request_missing_param(self, args):
        idx = args[0]
        # not sure whether reverse parsing could frame this
        if idx == 0:
            theme = "agent"
        elif idx == 1:
            theme = "patient"
        elif idx == 2:
            theme = "location"
        else:
            sys.exit("ERROR: unrecognized theme idx "+str(idx))
        #self.output.say("What is the " + theme + " of the action you'd like me to take?")
        
        # Using a bad approximation to generate an understandable prompt
        max_belief_action = ''
        if 'bring' in self.state.user_action_belief and 'at' in self.state.user_action_belief :
            if self.state.user_action_belief['bring'] > self.state.user_action_belief['at'] :
                max_belief_action = 'bring'
            else :
                max_belief_action = 'at'
        elif 'bring' in self.state.user_action_belief :
            max_belief_action = 'bring'
        else :
            max_belief_action = 'at'
        
        if max_belief_action == 'bring' :
            if idx == 0:
                self.output.say("What should I bring?")
            elif idx == 1:
                self.output.say("Whom should I bring something for?")    
        else :
            self.output.say("Where should I walk to?")
        
        u = self.input.get()
        
        if theme == 'agent' :
            theme = 'patient'
        if theme in self.parser_train_data :
            self.parser_train_data[theme].append(u)
        else :
            self.parser_train_data[theme] = [u]
        
        self.update_state_from_missing_param(u, idx)

    # update state after asking user to provide a missing parameter
    def update_state_from_missing_param(self, u, idx):
        self.state.update_requested_user_turn()

        # get n best parses for confirmation
        n_best_parses = self.parser.parse_expression(u, n=self.parse_depth)

        # try to digest parses to confirmation
        success = False
        for i in range(0, len(n_best_parses)):
            # print self.parser.print_parse(n_best_parses[i][0])  # DEBUG
            # print self.parser.print_semantic_parse_result(n_best_parses[i][1])  # DEBUG
            g = self.grounder.groundSemanticNode(n_best_parses[i][0], [], [], [])
            answer = self.grounder.grounding_to_answer_set(g)
            if len(answer) == 1:
                success = True
                self.state.update_from_missing_param(answer[0], idx, self.grounder)
                break
        if not success:
            self.state.update_from_failed_parse()

    # confirm a (possibly partial) action
    def confirm_action(self, args):
        a = args[0]
        # with reverse parsing, want to confirm(a.name(a.params))
        # for now, just use a.name and a.params raw

        #self.output.say("I should take action " + a.name+" involving " +
                        #','.join([str(p) for p in a.params if p is not None]) + "?")
        
        system_action = SystemAction('confirm_action', a.name)
        system_action.referring_params = dict()
        if a.name == 'bring' :
            if len(a.params) > 0 :
                system_action.referring_params['patient'] = a.params[0]
            if len(a.params) > 1 :
                system_action.referring_params['recipient'] = a.params[1]
            response = self.response_generator.get_sentence(system_action)    
            self.output.say(response)
        elif a.name == 'at' :
            if len(a.params) > 0 :
                system_action.referring_params['location'] = a.params[0]
            response = self.response_generator.get_sentence(system_action)    
            self.output.say(response)
        else :
            self.output.say("I should take action " + a.name+" involving " +
                        ','.join([str(p) for p in a.params if p is not None]) + "?")
                        
        c = self.input.get()
        self.update_state_from_action_confirmation(c, a)

    # update state after asking user to confirm a (possibly partial) action
    def update_state_from_action_confirmation(self, c, a):
        self.state.update_requested_user_turn()

        # get n best parses for confirmation
        n_best_parses = self.parser.parse_expression(c, n=self.parse_depth)

        # try to digest parses to confirmation
        success = False
        for i in range(0, len(n_best_parses)):
            # print self.parser.print_parse(n_best_parses[i][0])  # DEBUG
            # print self.parser.print_semantic_parse_result(n_best_parses[i][1])  # DEBUG
            if n_best_parses[i][0].idx == self.parser.ontology.preds.index('yes'):
                success = True
                self.state.update_from_action_confirmation(a, True)
            elif n_best_parses[i][0].idx == self.parser.ontology.preds.index('no'):
                success = True
                self.state.update_from_action_confirmation(a, False)
        if not success:
            self.state.update_from_failed_parse()

    # update state after asking user-initiative (open-ended) question about user goal
    def update_state_from_user_initiative(self, u):
        # print "Inside update_state_from_user_initiative"
        self.state.update_requested_user_turn()

        # get n best parses for utterance
        n_best_parses = self.parser.parse_expression(u, n=self.parse_depth)
        #print 'n_best_parses - ', n_best_parses

        # try to digest parses to action request
        success = False
        for i in range(0, len(n_best_parses)):
            #print "Digesting parse ", i
            #print self.parser.print_parse(n_best_parses[i][0])  # DEBUG
            #print self.parser.print_semantic_parse_result(n_best_parses[i][1])  # DEBUG
            success = self.update_state_from_action_parse(n_best_parses[i][0])
            if success:
                break
        # print "Finished trying to digest parses" 

        # no parses could be resolved into actions for state update
        if not success:
            # print "Has to recover from failed parse"
            self.state.update_from_failed_parse()

    # get dialog state under assumption that utterance is an action
    def update_state_from_action_parse(self, p):
        # print "Inside update_state_from_action_parse"         

        # get action, if any, from the given parse
        try:
            p_action = self.get_action_from_parse(p)
        except SystemError:
            p_action = None
        # print "Tried getting action from parse"
        if p_action is not None:
            # print "Going to update state from action"
            self.state.update_from_action(p_action, p, self.grounder)
            return True

        # if this failed, try again allowing missing lambdas to become UNK without token ties
        # print "Going to retry parsing"
        UNK_root = copy.deepcopy(p)
        curr = UNK_root
        heading_lambdas = []
        if curr.is_lambda and curr.is_lambda_instantiation:
            heading_lambdas.append(curr.lambda_name)
            curr = curr.children[0]
        if curr.children is not None:
            for i in range(0, len(curr.children)):
                if (curr.children[i].is_lambda and curr.children[i].lambda_name in heading_lambdas
                        and curr.type == self.parser.ontology.types.index('e')):
                    curr.children[i] = self.parser.create_unk_node()
        try:
            p_unk_action = self.get_action_from_parse(UNK_root)
        except SystemError:
            p_unk_action = None
        if p_unk_action is not None:
            self.state.update_from_action(p_unk_action, UNK_root, self.grounder)
            return True

        # parse could not be resolved into an action with which to update state
        return False

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
            pairs.append([t, r, Action.Action(a_name, a_params)])
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
                        a_candidate = Action.Action()
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

    def jointly_train_parser_and_generator_from_utterance_action_pairs(
            self, pairs, epochs=10, parse_beam=10, generator_beam=10):
        for e in range(0, epochs):
            print "training epoch "+str(e)
            random.shuffle(pairs)
            train_data = []
            num_actions_correct = 0
            num_tokens_correct = 0
            for t, r, a in pairs:
                generator_genlex = [self.generator, t, r]
                self.generator.flush_seen_nodes()
                n_best_parses = self.parser.parse_tokens(t, n=parse_beam, generator_genlex=generator_genlex)
                if len(n_best_parses) == 0:
                    print "WARNING: no parses found for tokens "+str(t)
                    continue
                a_chosen = None
                a_candidate = None
                correct_action_found = False
                for i in range(0, len(n_best_parses)):
                    try:
                        # print "parse: " + self.parser.print_parse(n_best_parses[i][0])  # DEBUG
                        # print self.parser.print_semantic_parse_result(n_best_parses[i][1])  # DEBUG
                        a_candidate = self.get_action_from_parse(n_best_parses[i][0])
                        # print "candidate: "+str(a_candidate)  # DEBUG
                    except SystemError:
                        a_candidate = Action.Action()
                    if i == 0:
                        a_chosen = a_candidate
                    if a_candidate.__eq__(a):
                        correct_action_found = True
                        self.parser.add_genlex_entries_to_lexicon_from_partial(n_best_parses[i][1])
                        break  # the action matches gold and this parse should be used in training
                if correct_action_found:
                    if a_chosen != a_candidate:
                        print "adding parsing training pair"  # DEBUG
                        print "chosen: "+str(n_best_parses[0])  # DEBUG
                        print "best: "+str(n_best_parses[i])  # DEBUG
                        train_data.append([t, n_best_parses[0], a_chosen, t, n_best_parses[i], a])
                    else:
                        num_actions_correct += 1
                else:
                    print "WARNING: could not find correct action '"+str(a)+"' for tokens "+str(t)
                # reverse parse based on semantic form rather than action, since from action we would need
                # to un-ground each atom as part of our potential expansions (a possible area of future work)
                n_best_sequences = self.generator.reverse_parse_semantic_form(r, c=generator_beam, n=generator_beam)
                if len(n_best_sequences) == 0:
                    print "WARNING: no sequences found for action "+str(a)+" with form "+self.parser.print_parse(r)
                    continue
                t_chosen = None
                t_chosen_str = None
                t_candidate_str = None
                correct_tokens_found = False
                t_str = ' '.join(t)
                for i in range(0, len(n_best_sequences)):
                    t_chosen = n_best_sequences[i][0]
                    t_candidate_str = ' '.join(t_chosen)
                    if i == 0:
                        t_chosen_str = t_candidate_str
                    if t_candidate_str == t_str:
                        correct_tokens_found = True
                        break
                if correct_tokens_found:
                    if t_chosen_str != t_candidate_str:
                        print "adding generation training pair"  # DEBUG
                        print "chosen: "+str(n_best_sequences[0])  # DEBUG
                        chosen_parse_like = [n_best_sequences[0][2][-1][0], n_best_sequences[0][1],
                                             n_best_sequences[0][2]]
                        print "best: "+str(n_best_sequences[i])  # DEBUG
                        correct_parse_like = [n_best_sequences[i][2][-1][0], n_best_sequences[i][1],
                                              n_best_sequences[i][2]]
                        train_data.append([t_chosen, chosen_parse_like, a, t, correct_parse_like, a])
                    else:
                        num_tokens_correct += 1
                else:
                    print "WARNING: could not find correct tokens "+str(t)+" for action '"+str(a)+"' with form " +\
                        self.parser.print_parse(r)
            print "\t"+str(num_actions_correct)+"/"+str(len(pairs))+" top parsing choices"
            print "\t"+str(num_tokens_correct)+"/"+str(len(pairs))+" top generation choices"
            if num_actions_correct == len(pairs) and num_tokens_correct == len(pairs):
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
                g = self.grounder.groundSemanticNode(arg, [], [], [])
                # print "Grounded"
                # print "arg: "+str(g)  # DEBUG
                answer = self.grounder.grounding_to_answer_set(g)
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
            return Action.Action(action, g_args)

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
                    
        #training_pairs = list()
        #for key in self.parser_train_data :
            #if key in answers :
                #for item in self.parser_train_data[key] :
                    #training_pairs.append((item, answers[key]))

        #print 'training_pairs = ', training_pairs

        #self.parser.train_learner_on_denotations(training_pairs, 10, 100, 3)

        #print 'Finished retraining parser'

        self.parser_train_data = dict()
        save_model(self.parser, 'static_parser')
