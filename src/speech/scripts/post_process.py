#!/usr/bin/python

"""
This script contains miscellaneous functions that
may be used to run certain types of post-processing
on different files. 

Usage
-----

To populate result file with semantic forms: ./post_process.py populate_semantic_forms [result_file_name] [test_file_name]

    Parameters:

        result_file_name    - The file containing the results which you would like to populate with
                              with semantic forms in the ground truth lines. 

        test_file_name      - The test file which was experimented with to generate the result files. 
                              This should contain the semantic forms needed for populating. 


Author: Rodolfo Corona, rcorona@utexas.edu
"""

import sys
import os

"""
This function goes through a result file from an experiment
and appends the semantic form for each phrase to its ground
truth. This may be used if a result file does not contain
semantic forms for any reason (so that experiments don't 
have to be re-performed). 
"""
def populate_result_file_with_semantic_forms(result_file_name, test_file_name):
    #Test file contains ground truth for phrases. 
    test_file = open(test_file_name, 'r')

    #Gets semantic form for all phrases in test file. 
    semantic_forms = {}

    for line in test_file: 
        phrase, semantic_form, _, _ = line.strip().split(';')

        semantic_forms[phrase] = semantic_form

    #Creates temp file to write changes to. 
    result_file = open(result_file_name, 'r')
    temp_file = open('temp.txt', 'w')

    for line in result_file: 
        #A '#' denotes a ground truth line for a phrase. 
        if line.startswith('#'):
            phrase = line.strip().split('#')[1].split(';')[0]

            #Ensures semantic forms were extracted correctly.
            if phrase not in semantic_forms:
                print 'Phrase ' + phrase + ' not in test file semantic forms!' 

                sys.exit()
            else:
                semantic_form = semantic_forms[phrase]

            #If for some reason already contains semantic form, then does not add it. 
            if not semantic_form in line:
                #Writes edited ground  truth line. 
                temp_file.write(line.strip() + ';' + semantic_forms[phrase] + '\n')
            else:
                temp_file.write(line)
        else:
            temp_file.write(line)

    #Moves temp file to overwrite result file. 
    os.rename('temp.txt', result_file_name)
    

def print_usage():
    print 'To populate result file with semantic forms: ./post_process.py populate_semantic_forms [result_file_name] [test_file_name]'

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'populate_semantic_forms':
        if len(sys.argv) == 4:
            populate_result_file_with_semantic_forms(sys.argv[2], sys.argv[3])
        else:
            print_usage()

    else:
        print_usage()
