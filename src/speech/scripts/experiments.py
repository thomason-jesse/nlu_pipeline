#!/usr/bin/python

"""
This script contains functions used for 
conducting ASR experiments, such as getting
the n-best hypothesis from Spinx, re-ranking,
etc. 

Usage
--------
ASR Nbest: ./experiments asr_n_best [sphinx_shared_library] [ac_model] [lm] [dict] [test_file] [nbest_file] [n]

    Parameters:
        sphinx_shared_library       - Path to the shared library which contains all the functions from sphinx, which
                                      this script wraps. 

        ac_model                    - Path to the acoustic model you wish to use. 

        lm                          - Path to the language model you would like to use. 

        dict                        - Path to the dictionary file you would like to use. 

        test_file                   - Path to the test file on which to perform the nbest hypothesis generation. 

        nbest_file                  - The file in which to store the nbest results. 

        n                           - The number of hypotheses to generate per phrase in the test file. 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import ctypes
import os
import sys

#Adds nlu_pipeline src folder in order to import modules from it. 
nlu_pipeline_path = '/home/rcorona/Desktop/UTCS/nlu_pipeline/src/'
sys.path.append(nlu_pipeline_path)

#Nlu pipeline modules.
try:
    import CKYParser
    from utils import *
except ImportError:
    print 'ERROR: Unable to load nlu_pipeline_modules! Verify that nlu_pipeline_path is set correctly!'
    sys.exit()



"""
This class is used to perform experiments with the
Sphinx ASR system, such as getting an n-best hypothesis list. 
"""
class SphinxExperiment:
    """
    Loads the pocketsphinx shared library
    in order to be able to use the system. 
    """
    def __init__(self, sphinx_lib):
        #Loads handle to library. 
        self.lib_handle = ctypes.CDLL(sphinx_lib)

        #Sets argument types for pocketsphinx functions. 
        self.lib_handle.sphinx_init.argtypes = (ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p)
        self.lib_handle.sphinx_close.argtypes = ()
        self.lib_handle.sphinx_n_best.argtypes = (ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int)

    """
    Initializes the pocketsphinx decoder using 
    a given acoustic model, language model, and
    dictionary. 
    """
    def init_decoder(self, ac_model, lm, dictionary):
        self.lib_handle.sphinx_init(ctypes.c_char_p(ac_model), ctypes.c_char_p(lm), ctypes.c_char_p(dictionary))

    """
    Closes decoder and cleans up memory. 
    """
    def close_decoder(self):
        self.lib_handle.sphinx_close()

    """
    Reads through a test file to get phrases and their 
    recordings. For each one gets n-best hypotheses using
    Sphinx and writes them to file along with the ground
    truth phrase. 
    """
    def n_best(self, test_file_name, nbest_result_file_name, n):
        #Converts in case it is still a string. 
        n = int(n)
    
        #Opens test file for reading. 
        test_file = open(test_file_name, 'r')

        #Creates nbest file before running speech recognition. 
        open(nbest_result_file_name, 'w').close()

        #Gets nbest hypotheses for each phrase in test file. 
        for line in test_file:
            #Gets phrase and pertinent recording file name from line. 
            phrase, semantic_form, denotation, recording_file_name = line.strip().split(';')

            #Formats ground truth to feed to Sphinx. 
            ground_truth = ';'.join([phrase, semantic_form, denotation])

            #Runs Sphinx ASR on it to get n-best hypotheses. 
            self.lib_handle.sphinx_n_best(ctypes.c_char_p(ground_truth), ctypes.c_char_p(recording_file_name), ctypes.c_char_p(nbest_result_file_name), ctypes.c_int(n))

###############################################################################
######################### Parser re-ranking functions #########################
###############################################################################

"""
This function takes in a string
and returns a tokenized version for 
it that is compatible with the format
the CKYParser expects. 
"""
def tokenize_for_parser(phrase):
    #Currently only splits possessive. 
    return phrase.replace("'", " '")

"""
This function takes a tokenized string
that was used by the CKYParser and
returns an un-tokenized version of it. 
"""
def un_tokenize_from_parser(phrase):
    return phrase.replace(" '", "'")

"""
Gets score of hypothesis element.
Used for re-ranking. 
"""
def get_hyp_score(hypothesis):
    return hypothesis[1]

"""
This method takes a file with n results
from speech recognition and re-ranks them
using the CKYParser class. 
"""
def re_rank_CKY(nbest_file_name, re_ranked_file_name, parser_path):
    nbest_file = open(nbest_file_name, 'r')

    #Loads parser. 
    parser = load_obj_general(parser_path)

    #Keeps track of data. 
    data = []
    hypotheses = None
    ground_truth = None
    parse_score = None

    #Keeps track of phrases which have already been scored to speed up process. 
    scores = {}

    for line in nbest_file:
        #Delimits new phrase. 
        if line.startswith('#'):
            #Adds last phrase and hypotheses if it exists. 
            if not (hypotheses == None or ground_truth == None or parse_score == None):
                data.append([ground_truth, hypotheses])

            ground_truth = line.strip().split('#')[1]
            
            #Starts new list of hypotheses. 
            hypotheses = []
        else:
            #Gets hypothesis and feeds it to parser to get top scoring parse score. 
            hypothesis = line.strip().split(';')[0]
            tokenized_hypothesis = tokenize_for_parser(hypothesis)

            #Hypotheses greater than 7 tokens in length are not considered. 
            if len(tokenized_hypothesis.split()) > 7:
                parse_score = float('-inf')
            elif tokenized_hypothesis in scores:
                parse_score = scores[tokenized_hypothesis]
            else:
                #Some times parser suffers an error. 
                try:
                    parse_score = parser.most_likely_cky_parse(tokenized_hypothesis).next()[1]
                except TypeError:
                    parse_score = float('-inf')
                    
                scores[tokenized_hypothesis] = parse_score
            
            hypotheses.append([hypothesis, parse_score])

    #Reranks list.
    for truth_hyp_list in data:
        truth_hyp_list[1] = sorted(truth_hyp_list[1], key=get_hyp_score, reverse=True)
        
    #Writes re-ranked hypotheses to new file. 
    re_ranked_file = open(re_ranked_file_name, 'w')
    
    for truth_hyp_list in data:
        #Writes ground truth phrase. 
        re_ranked_file.write('#' + truth_hyp_list[0] + '\n')

        #Writes hypotheses. 
        for hypothesis in truth_hyp_list[1]:
            re_ranked_file.write(hypothesis[0] + ';' + str(hypothesis[1]) + '\n')

    re_ranked_file.close()

#########################################################################################################


"""
Prints the usage for all the functions
conducting ASR experiments, such as getting
the n-best hypothesis from Spinx, re-ranking,
etc. 
"""
def print_usage():
    print 'ASR Nbest: ./experiments asr_n_best [sphinx_shared_library] [ac_model] [lm] [dict] [test_file] [nbest_file] [n]'
    print 'CKYParser re-rank: ./experiments parser_rerank [nbest_file] [re-ranked_file_name] [parser_path]'

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'asr_n_best':
        if not len(sys.argv) == 9:
            print_usage()
        else:
            sphinx = SphinxExperiment(sys.argv[2])
            sphinx.init_decoder(sys.argv[3], sys.argv[4], sys.argv[5])
            sphinx.nbest(sys.argv[6], sys.argv[7], sys.argv[8])
            sphinx.close_decoder()

    elif sys.argv[1] == 'parser_rerank':
        if not len(sys.argv) == 5:
            print_usage()
        else:
            re_rank_CKY(sys.argv[2], sys.argv[3], sys.argv[4])
