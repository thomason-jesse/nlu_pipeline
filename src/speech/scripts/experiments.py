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
        #Opens test file for reading. 
        test_file = open(test_file_name, 'r')

        #Creates nbest file before running speech recognition. 
        open(nbest_result_file_name, 'w').close()

        #Gets nbest hypotheses for each phrase in test file. 
        for line in test_file:
            #Gets phrase and pertinent recording file name from line. 
            phrase, _, _, recording_file_name = line.strip().split(';')

            #Runs Sphinx ASR on it to get n-best hypotheses. 
            self.lib_handle.sphinx_n_best(ctypes.c_char_p(phrase), ctypes.c_char_p(recording_file_name), ctypes.c_char_p(nbest_result_file_name), ctypes.c_int(n))

"""
Prints the usage for all the functions
offered by the script. 
"""
def print_usage():
    print 'ASR Nbest: ./experiments asr_n_best [sphinx_shared_library] [ac_model] [lm] [dict] [test_file] [nbest_file] [n]'

if __name__ == '__main__':
    if not len(sys.argv) == 9:
        print_usage()

    elif sys.argv[1] == 'asr_n_best':
        #Runs sphinx ASR experiment. 
        sphinx = SphinxExperiment(sys.argv[2])
        sphinx.init_decoder(sys.argv[3], sys.argv[4], sys.argv[5])
        sphinx.n_best(sys.argv[6], sys.argv[7], sys.argv[8])
        sphinx.close_decoder()

    else: 
        print_usage()
