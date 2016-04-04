#!/usr/bin/env python
__author__ = 'aishwarya'

import sys
import Ontology
import Lexicon
import KBGrounder
import CKYParser
import numpy, re

from Knowledge import Knowledge
from PomdpKtdqPolicy import PomdpKtdqPolicy
from PomdpTrainer import PomdpTrainer
from utils import *

# Fixing the random seed for debugging
#numpy.random.seed(4)

def create_trainer(parser_file, policy_file) :
    parser = load_model(parser_file)
    # Set parser hyperparams to best known values for test time
    parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
    parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
    parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
    parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried
    parser.max_expansions_per_non_terminal = 5 # max number of backpointers stored per nonterminal per cell in CKY chart

    grounder = KBGrounder.KBGrounder(parser.ontology)
    grounder.parser = parser

    knowledge = Knowledge()
    policy = load_model(policy_file)
    trainer = PomdpTrainer(parser, grounder, policy, param_mapping_file, vocab_mapping_file)
    return trainer

def train_only_parser(trainer, max_iterations=10, convergence_threshold=0.1)
    logs_dir = '/u/aish/Documents/Research/AMT_results/Batch2/corrected_pickle_logs/'
    theta = trainer.policy.theta
    P = trainer.policy.P
    
    for i in range(0, max_iterations) :
        train_from_new_logs(logs_dir, True, False, 'retraining/only_parser_policy', 'retraining/only_parser_parser') 
        save_model(trainer.parser, 'retraining/batch' + i + '_only_parser_parser')
        save_model(trainer.policy, 'retraining/batch' + i + '_only_parser_policy')
        new_theta = trainer.policy.theta
        new_P = trainer.policy.P
        theta_diff = numpy.sum(numpy.absolute(new_theta - theta))
        P_diff = numpy.sum(numpy.absolute(new_P - P))
        if theta_diff < convergence_threshold and P_diff < convergence_threshold :
            break
            
def train_only_dialog(trainer, max_iterations=10, convergence_threshold=0.1)
    logs_dir = '/u/aish/Documents/Research/AMT_results/Batch2/corrected_pickle_logs/'
    theta = trainer.policy.theta
    P = trainer.policy.P
    
    for i in range(0, max_iterations) :
        train_from_new_logs(logs_dir, False, True, 'retraining/only_dialog_policy', 'retraining/only_dialog_parser') 
        save_model(trainer.parser, 'retraining/batch' + i + '_only_dialog_parser')
        save_model(trainer.policy, 'retraining/batch' + i + '_only_dialog_policy')
        new_theta = trainer.policy.theta
        new_P = trainer.policy.P
        theta_diff = numpy.sum(numpy.absolute(new_theta - theta))
        P_diff = numpy.sum(numpy.absolute(new_P - P))
        if theta_diff < convergence_threshold and P_diff < convergence_threshold :
            break
            
def train_both(trainer, max_iterations=10, convergence_threshold=0.1)
    logs_dir = '/u/aish/Documents/Research/AMT_results/Batch2/corrected_pickle_logs/'
    theta = trainer.policy.theta
    P = trainer.policy.P
    
    for i in range(0, max_iterations) :
        train_from_new_logs(logs_dir, True, True, 'retraining/both_policy', 'retraining/both_parser') 
        save_model(trainer.parser, 'retraining/batch' + i + '_both_parser')
        save_model(trainer.policy, 'retraining/batch' + i + '_both_policy')
        new_theta = trainer.policy.theta
        new_P = trainer.policy.P
        theta_diff = numpy.sum(numpy.absolute(new_theta - theta))
        P_diff = numpy.sum(numpy.absolute(new_P - P))
        if theta_diff < convergence_threshold and P_diff < convergence_threshold :
            break

def main() :
    only_parser_trainer = create_trainer('only_parser_parser', 'only_parser_policy')
    only_parser_thread = Thread(target=train_only_parser, args=(only_parser_trainer,))
    only_parser_thread.daemon = True
    only_parser_thread.start()
    
    only_dialog_trainer = create_trainer('only_dialog_parser', 'only_dialog_policy')
    only_dialog_thread = Thread(target=train_only_dialog, args=(only_dialog_trainer,))
    only_dialog_thread.daemon = True
    only_dialog_thread.start()
    
    both_trainer = create_trainer('both_parser', 'both_policy')
    both_thread = Thread(target=train_both, args=(both_trainer,))
    both_thread.daemon = True
    both_thread.start()
    
    only_parser_thread.join()
    only_dialog_thread.join()
    both_thread.join()
            
if __name__ == '__main__' :
    main()            
    
    

