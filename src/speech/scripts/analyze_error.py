#!/usr/bin/python

"""
The functions in this script may
be used in order to conduct error
analysis on experiment results and
evaluations. 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import sys

#Path to nlu_pipeline modules. #TODO Change if needed. 
nlu_pipeline_path = '/home/rcorona/catkin_ws/src/nlu_pipeline/src/'
sys.path.append(nlu_pipeline_path)

#Nlu pipeline modules.
try:
    import CKYParser
    from utils import *
except ImportError:
    print 'ERROR: Unable to load nlu_pipeline_modules! Verify that nlu_pipeline_path is set correctly!'
    sys.exit()

"""
This function will print out
the parameters of a trained 
CKY Parser. 
"""
def dump_cky_parser_parameters(cky_parser_file):
    #Loads pickled parser. 
    parser = load_obj_general(cky_parser_file)

    #Prints parser parameters. 
    parser.print_parameters()

"""
Prints the usage options
for this script. 
"""
def print_usage():
    print 'Print parser parameters: ./analyze_error.py print_parser [parser_file]'


if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'print_parser':
        if len(sys.argv) == 3:
            dump_cky_parser_parameters(sys.argv[2])
        else:
            print_usage()

    else:
        print_usage()
