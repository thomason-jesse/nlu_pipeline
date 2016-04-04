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
#policy = PomdpKtdqPolicy(knowledge)
policy = load_model('policy_training/old_batch4')
print "instantiating Trainer"
param_mapping_file = 'src/nlu_pipeline/src/resources/old_ijcai_domain/ontology_mapping.csv'
vocab_mapping_file = 'src/nlu_pipeline/src/bootstrapping/Vocabulary.txt'
A = PomdpTrainer(parser, grounder, policy, param_mapping_file = param_mapping_file, vocab_mapping_file = vocab_mapping_file)

file_path = 'src/nlu_pipeline/src/models/policy_training/'
success_dir = '/u/aish/Documents/Research/rlg/logs_only/second/valid'
fail_dir = '/u/aish/Documents/Research/rlg/logs_only/second/invalid'

# Initialize using hand-coded policy
#A.init_weights_from_hand_coded_policy()
#file_name = file_path + 'hand_coded' + '.pkl'
#save_obj_general(A.policy, file_name)

# Train using old logs
#for i in range(0, 5) :
#    A.train_from_old_logs(success_dir, fail_dir)
#    file_name = file_path + 'old_batch' + str(i) + '.pkl'
#    save_obj_general(A.policy, file_name)

# Train using new logs
new_logs_dir = '/u/aish/Documents/Research/AMT_results/Batch1/all_corrected_pickle_logs/'
for i in range(0, 5) :
    A.train_from_new_logs(new_logs_dir)
    file_name = file_path + 'new_batch' + str(i) + '.pkl'
    save_obj_general(A.policy, file_name)
    

