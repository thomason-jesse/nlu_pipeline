__author__ = 'aishwarya'

import random
import copy
import sys, re
import Action

from SystemAction import SystemAction
from Utils import *
from UtteranceBasedStaticDialogState import UtteranceBasedStaticDialogState
from PomdpDialogAgent import PomdpDialogAgent

# This is logically an extension of DialogAgent but with type-checking,
# parser learning and the use of TemplateBasedGenerator to ask system 
# questions in a more natural form
# It inherits from PomdpDialogAgent rather than DialogAgent as there is
# functionality there that is useful to have inherited
class StaticDialogAgent(PomdpDialogAgent):

    def __init__(self, parser, grounder, policy, u_input, output, parse_depth=10):
        PomdpDialogAgent.__init__(self, parser, grounder, policy, u_input, output, parse_depth)    
        self.dialogue_stopped = False

    # initiate a new dialog with the agent with initial utterance u
    def initiate_dialog_to_get_action(self, u):
        self.dialogue_stopped = False
        self.parser_train_data = dict()
        self.parser_train_data['full'] = [u]

        self.state = UtteranceBasedStaticDialogState()
        
        self.cur_turn_log = [self.state, 'repeat_goal', u]
        self.dialog_objects_log = [self.cur_turn_log]

        self.update_state(u)

        # select next action from state
        action = None
        while action is None:
            self.cur_turn_log = [self.state]
            print "dialog state: "+str(self.state)  # DEBUG
            action = self.policy.resolve_state_to_action(self.state)

            # if the state does not clearly define a user goal, take a dialog action to move towards it
            if action is None:
                dialog_action, dialog_action_args = self.policy.select_dialog_action(self.state)
                self.cur_turn_log.append(str(dialog_action))
                if dialog_action not in self.dialog_actions:
                    sys.exit("ERROR: unrecognized dialog action '"+dialog_action+"' returned by policy for state "+str(self.state))
                self.state.previous_action = [dialog_action, dialog_action_args]
                self.dialog_action_functions[self.dialog_actions.index(dialog_action)](dialog_action_args)
                self.dialog_objects_log.append(self.cur_turn_log)
                if self.dialogue_stopped :
                    print 'Dialogue stopped'
                    return None
        
        complete_log_object = ('static', self.dialog_objects_log, action, self.parser_train_data)
        if self.dialog_objects_logfile is not None :
            save_obj_general(complete_log_object, self.dialog_objects_logfile)
        
        if action is not None :
            self.train_parser_from_dialogue(action)

        return action

    # request the user repeat their original goal
    def request_user_initiative(self, args):
        self.previous_system_action = SystemAction('repeat_goal')
        output = self.response_generator.get_sentence(self.previous_system_action)
        self.output.say(output)
        u = self.input.get()
        self.cur_turn_log = [self.previous_system_action, u]
        self.dialog_objects_log.append(self.cur_turn_log)
        if u.lower().strip() == 'stop' :
            self.dialogue_stopped = True
            return
        if 'full' in self.parser_train_data :
            self.parser_train_data['full'].append(u)
        else :
            self.parser_train_data['full'] = [u]
        self.update_state(u)

    # request a missing action parameter
    def request_missing_param(self, args):
        idx = args[0]
        # Using a bad approximation to generate an understandable prompt
        max_belief_action = None
        for goal in self.state.user_action_belief :
            if max_belief_action is None or self.state.user_action_belief[max_belief_action] < self.state.user_action_belief[goal] :
                max_belief_action = goal
        
        self.previous_system_action = SystemAction('request_missing_param', max_belief_action)
        param_order = self.knowledge.param_order[max_belief_action]
        self.previous_system_action.extra_data = param_order[idx]
        
        response = self.response_generator.get_sentence(self.previous_system_action)    
        self.output.say(response)
        u = self.input.get()
        self.cur_turn_log = [self.previous_system_action, u]
        self.dialog_objects_log.append(self.cur_turn_log)
        
        if u.lower().strip() == 'stop' :
            self.dialogue_stopped = True
            return
        
        theme = self.previous_system_action.extra_data
        if theme in self.parser_train_data :
            self.parser_train_data[theme].append(u)
        else :
            self.parser_train_data[theme] = [u]
        
        self.update_state(u)

    # confirm a (possibly partial) action
    def confirm_action(self, args):
        a = args[0]
        
        self.previous_system_action = SystemAction('confirm_action', a.name)
        self.previous_system_action.referring_params = dict()
        goal = a.name 
        if goal in self.knowledge.param_order :
            param_order = self.knowledge.param_order[goal]
            for (idx, param_name) in enumerate(param_order) :
                if len(a.params) > idx :
                    self.previous_system_action.referring_params[param_name] = a.params[idx]
        
        response = self.response_generator.get_sentence(self.previous_system_action)    
        self.output.say(response)
        c = self.input.get()
        self.cur_turn_log = [self.previous_system_action, c]
        self.dialog_objects_log.append(self.cur_turn_log)
        
        if c.lower().strip() == 'stop' :
            self.dialogue_stopped = True
            return
        
        self.update_state(c)

    
