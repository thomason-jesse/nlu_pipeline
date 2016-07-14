#!/usr/bin/python

"""
This script contains all functions
relevant to training the models used
on the speech corpus (i.e. parser, 
and asr training). 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

__author__ = ['rcorona', 'aishwarya']

import sys

#Adds path to parser scripts so that they may be imported.
#TODO CHANGE IF PATH CHANGES
sys.path.append('/scratch/cluster/rcorona/nlu_pipeline/src')

import Ontology
import Lexicon
import CKYParser
from utils import *

"""
Trains a new semantic parser using
the given training file and saves it
to the given parser_path using
a given ontology and lexicon. 
"""
def train_new_parser(parser_train_file, parser_path, ont_path, lex_path):
    #Instantiates ontology. 
    print "reading in Ontology"
    ont = Ontology.Ontology(ont_path)
    print "predicates: " + str(ont.preds)
    print "types: " + str(ont.types)
    print "entries: " + str(ont.entries)

    #Instantiates lexicon. 
    print "reading in Lexicon"
    lex = Lexicon.Lexicon(ont, lex_path)
    print "surface forms: " + str(lex.surface_forms)
    print "categories: " + str(lex.categories)
    print "semantic forms: " + str(lex.semantic_forms)
    print "entries: " + str(lex.entries)
    
    #Used to train new parser. 
    parser = CKYParser.CKYParser(ont, lex, use_language_model=True)
    
    #Used to train existing parser on new data. 
    #parser = load_obj_general(sys.argv[5])

    # Set parser hyperparams to best known values for test time
    parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
    parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
    parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
    parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried
    parser.max_expansions_per_non_terminal = 5 # max number of backpointers stored per nonterminal per cell in CKY chart

    # Read in train data. 
    data = parser.read_in_paired_utterance_semantics(parser_train_file)
    converged = parser.train_learner_on_semantic_forms(data, 10, reranker_beam=15)
    print '\n\n\nHand designed examples: converged = ', converged

    #Saves trained parser. 
    save_obj_general(parser, parser_path)

def print_usage():
    print 'To train new parser: ./train.py new_parser [parser_train_file] [parser_save_path] [ont_path] [lex_path]'

if __name__ == '__main__':
    if not len(sys.argv) >= 6:
        print_usage()
    
    #Train new parser case. 
    elif sys.argv[1] == 'new_parser':
        if not len(sys.argv) == 6:
            print_usage()
        else:
            train_new_parser(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
