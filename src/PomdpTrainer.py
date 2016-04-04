__author__ = 'aishwarya'

import sys, random, math, copy, ast, traceback, csv, itertools
from os import listdir
from os.path import isfile, join
import numpy as np
from sklearn import linear_model

from HISBeliefState import HISBeliefState
from SummaryState import SummaryState
from PomdpKtdqPolicy import PomdpKtdqPolicy
from SystemAction import SystemAction
from PomdpDialogAgent import PomdpDialogAgent
from utils import *

class FeatureWrapper :
    def __init__(self, init_arg=None) :
        self.feature_vector = None
        self.num_dialog_turns = 1
        if type(init_arg) == list :
            self.feature_vector = init_arg
        elif isinstance(init_arg, SummaryState) :
            self.feature_vector = init_arg.get_feature_vector()
    
    def get_feature_vector(self) :
        return self.feature_vector
    

# This class is supposed to read log files and convert them into a form 
# that can be used for training a policy. It requires the ability to 
# perform the NL understanding and belief monitoring.
# It extends PomdpDialogAgent for code reuse
class PomdpTrainer(PomdpDialogAgent) :

    def __init__(self, parser, grounder, policy, parse_depth=10, param_mapping_file=None, vocab_mapping_file=None):
        PomdpDialogAgent.__init__(self, parser, grounder, policy, None, None, parse_depth)
        if param_mapping_file is not None :
            # Map params from old IJCAI ontology to current ontology
            self.init_param_mapping(param_mapping_file)   
        if vocab_mapping_file is not None :
            self.init_vocab_mapping(vocab_mapping_file)

    def init_vocab_mapping(self, vocab_mapping_file) :
        f = open(vocab_mapping_file, 'r')
        reader = csv.reader(f, delimiter='\t')
        self.vocab_map = dict()
        for row in reader :
            if len(row) > 2 and len(row[2]) > 0 :
                key = row[0]
                value = row[2]
                self.vocab_map[key] = value 

    def map_text_to_desired_vocab(self, text) :
        tokens = self.parser.tokenize(text)[0]
        mapped_tokens = list()
        for token in tokens :
            mapped_token = token
            if self.vocab_map is not None and token in self.vocab_map :
                mapped_token = self.vocab_map[token]
            mapped_tokens.append(mapped_token)
        return ' '.join(mapped_tokens)

    def init_param_mapping(self, param_mapping_file) :
        f = open(param_mapping_file, 'r')
        reader = csv.reader(f)
        self.param_map = dict()
        for row in reader :
            key = row[0]
            value = row[1]
            if len(value) == 0 :
                value = None
            self.param_map[key] = value 
            
    def get_mapped_value(self, value) :
        if self.param_map is not None and value in self.param_map :
            return self.param_map[value]
        else :
            return None
            
    def test_ktdq_features(self) :
        b = [np.log(0.9), np.log(0.01), 1, 0, 1, 1, 'inform_full', 'at']
        print str(b)
        w = FeatureWrapper(b)
        f = self.policy.get_feature_vector(w, 'confirm_action')
            
    def init_weights_from_hand_coded_policy(self) :
        probs = [np.log(p) for p in [0, 0.00001, 0.2, 0.4, 0.6, 0.8, 1.0]]
        num_goals = [1 ,2 ,3]
        num_uncertain_params = [0, 1, 2, 3]
        last_utterance_type = ['inform_full', 'inform_param', 'affirm', 'deny']
        zero = [0]
        training_examples = list(itertools.product(probs, probs, num_goals, num_uncertain_params, zero, zero, last_utterance_type, zero))
        print 'len(training_examples) = ', len(training_examples)
        i = 1
        t = 0
        c = 0
        p = 0
        r = 0
        train_features = None
        train_outputs = None
        sample_weights = None
        to_print = []
        for example in training_examples :
            i += 1
            sample_weight = 1
            example_list = list(example)
            wrapper = FeatureWrapper(example_list)
            action = self.policy.get_action_from_hand_coded_policy(wrapper)
            if action == 'confirm_action' or action == 'take_action' :
                # These actions are rarer and need high sample weight to 
                # be learned
                sample_weight = 100
            
            for a in self.knowledge.summary_system_actions :
                new_feature_vector = self.policy.get_feature_vector(wrapper, a)
                if a == action :
                    value = 1
                else :
                    value = 0
                if train_features is None :
                    train_features = np.matrix(new_feature_vector).T
                    train_outputs = np.matrix(value).T
                    sample_weights = np.array(sample_weight)
                else :
                    train_features = np.vstack((train_features, np.matrix(new_feature_vector).T))
                    train_outputs = np.vstack((train_outputs, np.matrix(value).T))
                    sample_weights = np.hstack((sample_weights, sample_weight))
        train_features = np.tile(train_features, (5,1))
        train_outputs = np.tile(train_outputs, (5,1))
        sample_weights = np.tile(sample_weights, (1, 5))[0,:]
        model = linear_model.Ridge(alpha=0.5, fit_intercept=True)
        model.fit(train_features, train_outputs, sample_weights)
        preds = model.predict(train_features)
        print 'Train loss = ', sum(np.square(preds - train_outputs)) / train_features.shape[0]      # DEBUG
        self.policy.theta = model.coef_.T
        save_model(self.policy, 'ktdq_policy_object')
        
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
                if success_idx < len(success_files) :
                    fname = success_dir + '/' + success_files[success_idx]
                    self.train_policy_from_single_old_log(fname, True)
                    success_idx += 1
            else :
                if fail_idx < len(fail_files) :
                    fname = fail_dir + '/' + fail_files[fail_idx]
                    self.train_policy_from_single_old_log(fname, False)
                    fail_idx += 1
            save_model(self.policy, 'ktdq_policy_object')
        
        print 'Number of responses = ', self.num_turns_across_dialogs   # DEBUG
        print 'Number of responses that could not be parsed = ', self.num_not_parsed    # DEBUG
            
    # Train the policy using logs from the IJCAI 2015 experiment
    # This makes assumptions on the formatting of the logs
    def train_policy_from_single_old_log(self, conv_log_name, dialogue_successful=True) :
        conv_log = open(conv_log_name, 'r')
        self.max_prob_user_utterances = list()
        self.parser_train_data = dict()
        self.state = HISBeliefState(self.knowledge)
        for line in conv_log :
            parts = line.split('\t')
            print '---------------------------------------' # DEBUG
            print 'parts = ', parts # DEBUG
            print '---------------------------------------' # DEBUG
            if parts[0] == 'USER' : 
                if self.previous_system_action is not None :
                    try :
                        prev_state = SummaryState(self.state)
                        response = parts[1]
                        response = self.map_text_to_desired_vocab(response)
                        print 'Updating state for response :', response # DEBUG
                        self.update_state(response)
                        next_state = SummaryState(self.state)
                        print '---------------------------------------' # DEBUG
                        print '---------------------------------------' # DEBUG
                        print 'prev_state : ', str(prev_state)  # DEBUG
                        print '---------------------------------------'
                        print 'action : ', str(self.previous_system_action) # DEBUG
                        print '---------------------------------------' # DEBUG
                        print 'next_state : ', str(next_state)  # DEBUG
                        print '---------------------------------------' # DEBUG
                        self.policy.train(prev_state, self.previous_system_action.action_type, next_state, self.knowledge.per_turn_reward)
                        self.previous_system_action = None
                    except :
                        self.num_not_parsed += 1
                        print "Exception in user code:" # DEBUG
                        print '-'*60    # DEBUG
                        traceback.print_exc(file=sys.stdout)    # DEBUG
                        print '-'*60    # DEBUG
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
                            params['location'] = self.get_mapped_value(details[2].strip())
                    elif details[0] == 'served' :
                        goal = 'bring'
                        if details[1] is not None :
                            params['patient'] = self.get_mapped_value(details[1].strip())
                        if details[2] is not None :
                            params['recipient'] = self.get_mapped_value(details[2].strip())
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
                print '---------------------------------------' # DEBUG
                print 'action : ', str(self.previous_system_action) # DEBUG
                print '---------------------------------------' # DEBUG
        
        # Need to feed it with terminal reward
        prev_state = SummaryState(self.state)
        if dialogue_successful :
            self.policy.train(prev_state, 'take_action', None, self.knowledge.correct_action_reward)
        else :
            self.policy.train(prev_state, 'take_action', None, self.knowledge.wrong_action_reward)
    
    # Train the policy using pickle logs from new experiment
    def train_from_new_logs(self, log_dir, train_parser=False, train_policy=False, policy_save_name='ktdq_policy_object', parser_save_name='parser') :
        self.num_turns_across_dialogs = 0
        self.num_not_parsed = 0
        
        files = [f for f in listdir(log_dir) if isfile(join(log_dir, f))]
        random.shuffle(files)
        idx = 0

        err_log = open('src/nlu_pipeline/src/Pomdp_training_errors.txt', 'w')

        for idx in range(0, len(files)) :
            try : 
                fname = log_dir + files[idx]
                self.train_policy_and_parser_from_single_new_log(fname, train_parser, train_policy)
                save_model(self.policy, policy_save_name)
                save_model(self.parser, parser_save_name)
            except KeyboardInterrupt, SystemExit :
                raise
            except :
                err_log.write('Error when training on file :' + log_dir + files[idx])
                err_log.write(traceback.format_exc() + '\n\n\n')
                err_log.flush()

        err_log.close()        
        print 'Number of responses = ', self.num_turns_across_dialogs   # DEBUG
        print 'Number of responses that could not be parsed = ', self.num_not_parsed    # DEBUG
    
        
    # Train from new log files
    def train_policy_and_parser_from_single_new_log(self, conv_log_name, train_parser=False, train_policy=False) :
        self.retrain_parser = False # This is not going to run like a 
                                    # normal dialog so retraining will be 
                                    # done explicitly
        log = load_obj_general(conv_log_name)
        print log
        
        # log is a tuple of
        #   (agent_type, [(system_action, response)...], final_action, parser_train_data)
        #   agent_type = 'pomdp' or 'static'
        #   system_action can be of type SystemAction or a string 'take_action'
        #   response is either a string or None
        relevant_log_type = None
        if not train_parser and not train_policy :
            return
        elif train_parser and not train_policy :
            relevant_log_type = 'only_parser_learning'
        elif not train_parser and train_policy :
            relevant_log_type = 'only_dialog_learning'
        else :
            relevant_log_type = 'both_parser_and_dialog_learning'
        
        # KTDQ is an off policy algorithm so it can be trained on data 
        # from both pomdp and static agents. However, since the experiment
        # compares the agents, we want to train it only from pomdp instances
        if log[0] == relevant_log_type :
            if train_policy :
                self.state = HISBeliefState(self.knowledge)
                
                # For each system_action, response pair, update state and
                # train policy with standard per turn reward
                for [system_action, response] in log[1] :
                    prev_state = SummaryState(self.state)
                    next_state = None
                    action_type = system_action
                    if isinstance(system_action, SystemAction) :
                        self.previous_system_action = system_action
                        self.update_state(response)
                        next_state = SummaryState(self.state)
                        action_type = system_action.action_type
                    elif type(system_action) == 'str' and system_action == 'repeat_goal' :
                        self.previous_system_action = SystemAction('repeat_goal')
                        self.update_state(response)
                        next_state = SummaryState(self.state)
                        action_type = system_action.action_type
                    self.policy.train(prev_state, action_type, next_state, self.knowledge.per_turn_reward)
                
            if log[3] : # Dialogue was successful
                if train_policy :
                    # Train policy from success of dialogue    
                    self.policy.train(prev_state, 'take_action', None, self.knowledge.correct_action_reward)
                
                # Retrain parser
                if train_parser :
                    self.parser_train_data = log[4]
                    self.train_parser_from_dialogue(log[2])
            else :
                if train_policy :
                    # Train policy from failure of dialogue
                    self.policy.train(prev_state, 'take_action', None, self.knowledge.wrong_action_reward)
            
                
                    
                    
                    
                                     
        
