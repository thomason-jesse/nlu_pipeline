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

        semantic_forms[phrase] = 'M : ' + add_type_constraints(semantic_form.replace('M : ', ''))

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

            #Writes edited ground  truth line. 
            temp_file.write('#' + phrase + ';' + semantic_forms[phrase] + '\n')
        else:
            temp_file.write(line)

    #Moves temp file to overwrite result file. 
    os.rename('temp.txt', result_file_name)

def add_type_constraints(semantic_form):
    #Gets action and generates typed semantic form as necessary.
    action = semantic_form.split('(')[0]

    if action == 'walk':
        first_element = semantic_form.split('(')[1]

        #If not 'the', then must be a location, so keep semantic form intact. 
        if not first_element == 'the':
            new_semantic_form = semantic_form
                
        #Otherwise, is predicate, so replace generic e type with l (for location). 
        else:
            new_semantic_form = semantic_form.replace('lambda x:e', 'lambda x:l', 1)

            #Currently only expects one lambda, so assert that this is in fact true. 
            if 'lambda x:e' in new_semantic_form:
                print "More than one lambda expression in walk command!: " + semantic_form + '\n'
                sys.exit()

    elif action == 'searchroom':
        new_semantic_form = semantic_form

        #Currently expects no lambda expressions, so assert this. 
        if 'lambda' in new_semantic_form:
            print "Lambda expression found in 'searchroom' command!: " + new_semantic_form + '\n'
            sys.exit()

    elif action == 'bring':
        if 'lambda' in semantic_form:
            #Replace generic 'e' type with item type 'i'. 
            new_semantic_form = semantic_form.replace('lambda x:e', 'lambda x:i', 1)

            #Currently only expects one lambda expression, so assert this. 
            if 'lambda x:e' in new_semantic_form:
                print "More than one lambda expression found in bring command!: " + semantic_form + '\n'
                sys.exit()
        else:
            new_semantic_form = semantic_form
    else:
        print "Unaccounted for action!: " + action + '\n'
        sys.exit()
 
    #Returns typed semantic form.
    return new_semantic_form

def add_type_constraints_to_file(file_name): 
    old_file = open(file_name, 'r')
    tmp_file = open('temp.txt', 'w')

    for line in old_file:
        #Must be a parser training file. 
        if not line.startswith("M : "):
            tmp_file.write(line)
        else:
            #Gets generic semantic form. 
            new_semantic_form = add_type_constraints(line.split('M : ')[1].strip()) 
           
            #Writes new semantic form to new file. 
            tmp_file.write('M : ' + new_semantic_form + '\n')

    tmp_file.close()

    #Overwrites old file with new one, now containing type constraints. 
    os.rename('temp.txt', file_name)

def print_usage():
    print 'To populate result file with semantic forms: ./post_process.py populate_semantic_forms [result_file_name] [test_file_name]'
    print 'To add type constraints to semantic forms in a file: ./post_process.py type_constraints [file_name]'

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'populate_semantic_forms':
        if len(sys.argv) == 4:
            populate_result_file_with_semantic_forms(sys.argv[2], sys.argv[3])
        else:
            print_usage()

    elif sys.argv[1] == 'type_constraints':
        if len(sys.argv) == 3:
            add_type_constraints_to_file(sys.argv[2])
        else:
            print_usage()

    else:
        print_usage()
