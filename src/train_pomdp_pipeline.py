#!/usr/bin/env python
__author__ = 'aishwarya'

import sys
import Ontology
import Lexicon
import KBGrounder
import CKYParser
import numpy, re, csv

from Knowledge import Knowledge
from PomdpKtdqPolicy import PomdpKtdqPolicy
from PomdpTrainer import PomdpTrainer
from utils import *

# Fixing the random seed for debugging
numpy.random.seed(4)

print "reading in Ontology"
ont = Ontology.Ontology(sys.argv[1])
print "predicates: " + str(ont.preds)
print "types: " + str(ont.types)
print "entries: " + str(ont.entries)

print "reading in Lexicon"
lex = Lexicon.Lexicon(ont, sys.argv[2])
print "surface forms: " + str(lex.surface_forms)
print "categories: " + str(lex.categories)
print "semantic forms: " + str(lex.semantic_forms)
print "entries: " + str(lex.entries)

print "instantiating KBGrounder"
grounder = KBGrounder.KBGrounder(ont)

print "instantiating Parser"
#parser = CKYParser.CKYParser(ont, lex, use_language_model=True)
## Set parser hyperparams to best known values for training
#parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
#parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
#parser.max_cky_trees_per_token_sequence_beam = 1000  # for tokenization of an utterance, max cky trees considered
#parser.max_hypothesis_categories_for_unknown_token_beam = 5  # for unknown token, max syntax categories tried
#parser.max_expansions_per_non_terminal = 5 # max number of backpointers stored per nonterminal per cell in CKY chart
#d = parser.read_in_paired_utterance_semantics(sys.argv[3])
#converged = parser.train_learner_on_semantic_forms(d, 10, reranker_beam=10)
#if not converged:
    #raise AssertionError("Training failed to converge to correct values.")
#save_model(parser, 'parser')
parser = load_model('parsers/parser_1500')

# Set parser hyperparams to best known values for test time
parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried
parser.max_expansions_per_non_terminal = 5 # max number of backpointers stored per nonterminal per cell in CKY chart

grounder.parser = parser
grounder.ontology = parser.ontology

knowledge = Knowledge()
policy = PomdpKtdqPolicy(knowledge)
print "instantiating Trainer"
param_mapping_file = 'src/nlu_pipeline/src/resources/old_ijcai_domain/ontology_mapping.csv'
vocab_mapping_file = 'src/nlu_pipeline/src/bootstrapping/Vocabulary.txt'
trainer = PomdpTrainer(parser, grounder, policy, param_mapping_file = param_mapping_file, vocab_mapping_file = vocab_mapping_file)

file_path = 'src/nlu_pipeline/src/models/policy_training/'
success_dir = '/u/aish/Documents/Research/rlg/logs_only/second/valid'
fail_dir = '/u/aish/Documents/Research/rlg/logs_only/second/invalid'

# Initialize using hand-coded policy
trainer.init_weights_from_hand_coded_policy()
file_name = file_path + 'hand_coded' + '.pkl'
save_obj_general(trainer.policy, file_name)

param_diff_file = 'src/nlu_pipeline/src/bootstrapping/param_diffs.csv'
param_diff_file_handle = open(param_diff_file, 'a')
param_diff_file_writer = csv.writer(param_diff_file_handle, delimiter=',')
param_diff_file_writer.writerow(['name', 'theta_diff', 'P_diff'])

theta = trainer.policy.theta
P = trainer.policy.P

# Train using old logs
for i in range(0, 5) :
    trainer.train_from_old_logs(success_dir, fail_dir)
    file_name = file_path + 'old_batch_iteration' + str(i) + '.pkl'
    save_obj_general(trainer.policy, file_name)
    
    new_theta = trainer.policy.theta
    new_P = trainer.policy.P
    theta_diff = numpy.sum(numpy.absolute(new_theta - theta))
    P_diff = numpy.sum(numpy.absolute(new_P - P))
    param_diff_file_writer.writerow(['old_batch_iteration' + str(i), theta_diff, P_diff])
    theta = new_theta
    P = new_P

# Train using new logs
new_logs_dirs = ['/u/aish/Documents/Research/AMT_results/before_fluency_verification/Batch0/all_corrected_pickle_logs/', \
    '/u/aish/Documents/Research/AMT_results/before_fluency_verification/Batch1/corrected_pickle_logs/', \
    '/u/aish/Documents/Research/AMT_results/before_fluency_verification/Batch2/corrected_pickle_logs/', \
    '/u/aish/Documents/Research/AMT_results/testing_fluency/corrected_pickle_logs/', \
    '/u/aish/Documents/Research/AMT_results/after_fluency/Batch0/corrected_pickle_logs/', \
    '/u/aish/Documents/Research/AMT_results/after_fluency/Batch1/corrected_pickle_logs/', \
    '/u/aish/Documents/Research/AMT_results/after_fluency/Batch2/corrected_pickle_logs/', \
    ]
    
for i in range(0, 5) :
    for (j, log_dir) in enumerate(new_logs_dirs) :
        trainer.train_policy_from_new_logs(log_dir)
        file_name = file_path + 'new_batch' + str(j) + '_iteration' + str(i) + '.pkl'
        save_obj_general(trainer.policy, file_name)
        
        new_theta = trainer.policy.theta
        new_P = trainer.policy.P
        theta_diff = numpy.sum(numpy.absolute(new_theta - theta))
        P_diff = numpy.sum(numpy.absolute(new_P - P))
        param_diff_file_writer.writerow(['new_batch' + str(j) + '_iteration' + str(i), theta_diff, P_diff])
        theta = new_theta
        P = new_P
    

