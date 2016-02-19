__author__ = 'aishwarya'

import random
import copy
import sys, re
import Action
from TemplateBasedGenerator import TemplateBasedGenerator
from SystemAction import SystemAction
from Knowledge import Knowledge
from Utils import *
from StaticDialogStateWithTypechecking import StaticDialogStateWithTypechecking
from DialogAgent import DialogAgent

# Extends DialogAgent and adds type-checking, parser learning and uses 
# TemplateBasedGenerator to ask system questions in a more natural form
class StaticDialogAgent(DialogAgent):

    def __init__(self, parser, grounder, policy, u_input, output, parse_depth=10):
        DialogAgent.__init__(self, parser, grounder, policy, u_input, output, parse_depth)    

        # To convert system actions to understandable English
        self.response_generator = TemplateBasedGenerator() 
        
        self.knowledge = Knowledge()    # For typechecking information
        self.dialogue_stopped = False
        
        # Logging
        self.final_action_log = None
        self.lexical_addition_log = None

    # initiate a new dialog with the agent with initial utterance u
    def initiate_dialog_to_get_action(self, u):
        self.dialogue_stopped = False
        self.parser_train_data = dict()
        #print "Function call succeeded"         

        #self.state = StaticDialogState.StaticDialogState()
        self.state = StaticDialogStateWithTypechecking(grounder)
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
                if self.dialogue_stopped :
                    print 'Dialogue stopped'
                    return None
        
        if action is not None :
            self.train_parser_from_dialogue(action)

        return action

    # request the user repeat their original goal
    def request_user_initiative(self, args):
        self.output.say("Sorry I couldn't understand that. Could you reword your original request?")
        u = self.input.get()
        if u.lower().strip() == 'stop' :
            self.dialogue_stopped = True
            return
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
        max_belief_action = None
        for goal in self.state.user_action_belief :
            if max_belief_action is None or self.state.user_action_belief[max_belief_action] < self.state.user_action_belief[goal] :
                max_belief_action = goal
        
        if max_belief_action == 'bring' :
            if idx == 0:
                self.output.say("What should I bring?")
            elif idx == 1:
                self.output.say("Whom should I bring something for?")    
        elif max_belief_action == 'at' :
            self.output.say("Where should I walk to?")
        elif max_belief_action == 'searchroom' :
            if idx == 0:
                self.output.say("Whom should I search for?")
            elif idx == 1:
                self.output.say("Where should I search?")    
        else :
            self.output.say("What is the " + theme + " of the action you want me to take?")    
        
        u = self.input.get()
        
        if u.lower().strip() == 'stop' :
            self.dialogue_stopped = True
            return
        
        if theme == 'agent' :
            theme = 'patient'
        if theme in self.parser_train_data :
            self.parser_train_data[theme].append(u)
        else :
            self.parser_train_data[theme] = [u]
        
        self.update_state_from_missing_param(u, idx)

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
        elif a.name == 'searchroom' :
            if len(a.params) > 0 :
                system_action.referring_params['patient'] = a.params[0]
            if len(a.params) > 1 :
                system_action.referring_params['location'] = a.params[1]
            response = self.response_generator.get_sentence(system_action)    
            self.output.say(response)
        else :
            self.output.say("I should take action " + a.name+" involving " +
                        ','.join([str(p) for p in a.params if p is not None]) + "?")
                        
        c = self.input.get()
        
        if c.lower().strip() == 'stop' :
            self.dialogue_stopped = True
            return
        
        self.update_state_from_action_confirmation(c, a)

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
        
        ## Add new entries to lexicon
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

        self.parser_train_data = dict()
        save_model(self.parser, 'static_parser')
