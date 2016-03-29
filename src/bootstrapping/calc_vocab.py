#!/usr/bin/env python
__author__ = 'aishwarya'

import sys, re
from os import listdir
from os.path import isfile, join

sys.path.append('../')
from utils import *


def calc_vocab(folders, parser) :
    vocabulary = dict()
    for folder in folders :
        files = [f for f in listdir(folder) if isfile(join(folder, f))]
        for file_name in files :
            full_file_name = folder + '/' + file_name
            file_handle = open(full_file_name, 'r')
            for line in file_handle :
                parts = line.split('\t')
                if len(parts) > 1 :
                    text = parts[1]
                    text = re.sub("'s", " 's", text)
                    tokens = parser.tokenize(text)[0]
                    for token in tokens :
                        if token not in vocabulary :
                            vocabulary[token] = 0
                        vocabulary[token] = vocabulary[token] + 1
    pairs = vocabulary.items()
    pairs.sort()
    for (word, count) in pairs :
        print word + '\t' + str(count)
        
if __name__ == '__main__' :
    parser_file = '../models/parsers/parser_1000.pkl'
    parser = load_obj_general(parser_file)

    # Set parser hyperparams to best known values for test time
    parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
    parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
    parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
    parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried
    parser.max_expansions_per_non_terminal = 5 # max number of backpointers stored per nonterminal per cell in CKY chart

    #folders = ['/u/aish/Documents/Research/rlg/logs_only/second/valid',
        #'/u/aish/Documents/Research/rlg/logs_only/second/invalid']
    folders = ['/u/aish/Documents/Research/rlg/logs_only/second/invalid']
    calc_vocab(folders, parser)
                
            
            
