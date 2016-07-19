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
import subprocess

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

"""
Trains a new language model using
the given training file and saves
it to the given lm_path. 
"""
def train_new_lm(lm_train_file, lm_path):
    #Path to executable used to create language model. TODO Change if needed.
    srilm_path = '../bin/SRILM/bin/i686-m64/ngram-count'
    
    #Arguments to call it. 
    args = [srilm_path, '-wbdiscount', '-interpolate', '-text', lm_train_file, '-lm', lm_path]

    #Calls it to create lm. 
    subprocess.call(args)

def adapt_accoustic_model(acoustic_model_path, corpus_folder, recordings_dir):
    #Paths needed for execution. 
    bin_path = '../bin'
    sphinxbase_lib_path = '../resources/sphinxbase_install/lib/'

    #Sets LD_LIBRARY_PATH for needed libraries. 
    os.environ['LD_LIBRARY_PATH'] = '$LD_LIBRARY_PATH:' + sphinxbase_lib_path

    #Composes arguments for feature extraction. 
    args = [bin_path + '/sphinx_fe']
    args.append('-argfile')
    args.append(acoustic_model_path + '/feat.params')
    args.append('-samprate')
    args.append('16000')
    args.append('-c')
    args.append(corpus_folder + '/asr_training/train.fileids')
    args.append('-di')
    args.append(recordings_dir)
    args.append('-do')
    args.append(corpus_folder + '/models/acc_model')
    args.append('-ei')
    args.append('raw')
    args.append('-eo')
    args.append('mfc')
    args.append('-raw')
    args.append('yes')

    #Extracts features from adaptation recordings. 
    subprocess.call(args)

def print_usage():
    print 'To train new parser: ./train.py new_parser [parser_train_file] [parser_save_path] [ont_path] [lex_path]'
    print 'To train new language model: ./train.py new_lm [lm_train_file] [lm_save_path]'
    print 'To adapt an accoustic model: ./train.py adapt_acc_model [accoustic_model_path] [corpus_folder] [recordings_dir]'

if __name__ == '__main__':
    if not len(sys.argv) >= 4:
        print_usage()
    
    #Train new parser case. 
    elif sys.argv[1] == 'new_parser':
        if not len(sys.argv) == 6:
            print_usage()
        else:
            train_new_parser(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])

    #Train a new language model. 
    elif sys.argv[1] == 'new_lm':
        if len(sys.argv) == 4:
            train_new_lm(sys.argv[2], sys.argv[3])
        else:
            print_usage()

    #Adapt an accoustic model. 
    elif sys.argv[1] == 'adapt_acc_model':
        if len(sys.argv) == 5:
            adapt_accoustic_model(sys.argv[2], sys.argv[3], sys.argv[4])
        else:
            print_usage()
