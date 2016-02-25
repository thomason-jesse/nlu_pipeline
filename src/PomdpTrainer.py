__author__ = 'aishwarya'

import sys, random, math, copy, ast
from os import listdir
from os.path import isfile, join
import numpy as np

from SummaryState import SummaryState
#from PomdpGpSarsaPolicy import PomdpGpSarsaPolicy
from PomdpKtdqPolicy import PomdpKtdqPolicy
from SystemAction import SystemAction
from PomdpDialogAgent import PomdpDialogAgent
from utils import *

# This class is supposed to read log files and convert them into a form 
# that can be used for training a policy. It requires the ability to 
# perform the NL understanding and belief monitoring.
# It extends PomdpDialogAgent for code reuse
class PomdpTrainer(PomdpDialogAgent) :

    def __init__(self, parser, grounder, policy, parse_depth=10):
        PomdpDialogAgent.__init__(self, parser, grounder, policy, None, None, parse_depth)

    # Train the policy using logs from the IJCAI 2015 experiment
    # This makes assumptions on the formatting of the logs
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
            save_model(policy, 'ktdq_policy_object')
            
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
        
