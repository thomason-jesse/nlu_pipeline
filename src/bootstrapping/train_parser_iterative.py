#!/usr/bin/env python
__author__ = 'aishwarya'

import random
import sys

sys.path.append('../')
import Ontology
import Lexicon
import CKYParser
from utils import *

if __name__ == '__main__' :
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
    
    parser = CKYParser.CKYParser(ont, lex, use_language_model=True)
    
    # Set parser hyperparams to best known values for training
    parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
    parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
    parser.max_cky_trees_per_token_sequence_beam = 1000  # for tokenization of an utterance, max cky trees considered
    parser.max_hypothesis_categories_for_unknown_token_beam = 5  # for unknown token, max syntax categories tried
    parser.max_expansions_per_non_terminal = 5  # max number of backpointers stored per nonterminal per cell in CKY chart

    file_path = '../models/parser_training/'

    # Read in easy train data 
    easy_data = parser.read_in_paired_utterance_semantics(sys.argv[3])
    converged = parser.train_learner_on_semantic_forms(easy_data, 10, reranker_beam=10)
    print '\n\n\nHand designed examples: converged = ', converged

    file_name = file_path + 'basic.pkl'
    save_obj_general(parser, file_name)
    
    hard_data = parser.read_in_paired_utterance_semantics(sys.argv[4])
    usable_data = list()
    for (utterance, semantic_form) in hard_data :
        tokens = parser.tokenize(utterance)[0]
        if len(tokens) <= 7 :
            usable_data.append((utterance, semantic_form))
    for i in [10 * j for j in range(0, 100)] :
        if i >= len(usable_data) :
            break
        train_data = usable_data[i:i+10]
        converged = parser.train_learner_on_semantic_forms(train_data, 10, reranker_beam=10)
        print '\n\n\nBatch ', i/10, ': converged = ', converged    
        file_name = file_path + 'batch' + str(i/10) + '.pkl'
        save_obj_general(parser, file_name)
    
    file_name = '../models/parser_1000_iterative.pkl'
    save_obj_general(parser, file_name)
