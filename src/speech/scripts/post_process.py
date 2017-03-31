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
import re
import pickle

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

"""
This goes through a file and extracts all of the items being described
that could be grounded to in a corpus. 
"""
def find_groundable_items(corpus_folder, pickle_file_name):
    #Will keep all unique items. 
    adjective_items = []
    single_items = set()

    #Expression used to find properties. 
    regex = re.compile('[a-z]+\(x\)')

    #Looks for items in each of the corpus' files. 
    for file_name in os.listdir(corpus_folder):

        #Only processes it if valid type. 
        if file_name.endswith('.usr'):
            for line in open(corpus_folder + '/' + file_name, 'r'):
                #Unpacks  line. Removes category from semantic form. 
                phrase, sem_form, denotation, recording = line.strip().split(';')
                sem_form = ':'.join(sem_form.split(':')[1:]).strip()

                #Items are only found in bring action.
                if 'bring' in sem_form: 
                    properties = re.findall(regex, sem_form)

                    #Ensures it has properties before adding. 
                    if len(properties) > 0:
                        properties = set(properties)

                        #Checks to see if item with these properties exists. 
                        exists = False

                        for item in adjective_items: 
                            if item == properties:
                                exists = True

                        #If new item, then add it to items. 
                        if not exists:
                            adjective_items.append(properties)

                    else:
                        single_items.add(sem_form.split('bring(')[1].split(',')[0])

    #Now makes items for grounder. 
    items = {}
    counter = 0

    #Adjective described have properties. 
    for item in adjective_items:
        items['item' + str(counter)] = item
        counter += 1

    #Single items don't, but have a name. 
    for item in single_items:
        items[item] = None

    #Keeps kb of item properties. 
    kb = {}

    #Populates kb. 
    for item in items:
       if not items[item] == None: 
            for pred in items[item]:
                #Adds item to predicate's list. 
                if pred in kb:
                    kb[pred].append(item)
                else:
                    kb[pred] = [item]

    for pred in kb:
        print str(pred) + ': ' + str(kb[pred])

    #Adds in known atoms. 
    kb['possesses'] = [('scott', 'l3_404'), ('ray', 'l3_512'), ('dana', 'l3_510'), ('peter', 'l3_508'), ('shiqi', 'l3_432'), ('jivko', 'l3_420'),
                       ('stacy', 'l3_502'), ('jesse', 'l3_414b'), ('aishwarya', 'l3_414b'), ('rodolfo', 'l3_414b')]

    kb['office'] = ['l3_404', 'l3_512', 'l3_510', 'l3_508', 'l3_432', 'l3_420', 'l3_502', 'l3_414b']


    for pred in kb:
        print str(pred) + ': ' + str(kb[pred])

    #Now pickles the kb. 
    pickle.dump(kb, open(pickle_file_name, 'w'))

def is_int(string): 
    try: 
        int(string)
        return True
    except ValueError:
        return False

def convert_numbers(phrase): 
    numbers = {'zero': 0, 'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5,
               'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
               'twelve': 12, 'fourteen': 14, 'sixteen': 16, 'eighteen': 18, 
               'twenty': 20, 'thirty-two': 32, 'thirty-four': 34, 'thirty-five': 35}

    tokenized_phrase = phrase.split()
    new_phrase = []

    #Replace each instance of an alphabetical number in phrase with its numeric counterpart. 
    for word in tokenized_phrase:
        if word in numbers:
            new_phrase.append(str(numbers[word]))
        else:
            new_phrase.append(word)

    tokenized_phrase = new_phrase
    new_phrase = []
    index = 0

    #Remove spaces between numbers since they should most likely all be the same number. 
    while index < len(tokenized_phrase):
        if is_int(tokenized_phrase[index]):
            number_str = ''
            
            while index < len(tokenized_phrase) and is_int(tokenized_phrase[index]):
                number_str += tokenized_phrase[index]
                index += 1

            new_phrase.append(number_str)

        else:
            new_phrase.append(tokenized_phrase[index])
            index += 1

    return ' '.join(new_phrase).strip()

def convert_numbers_corpus(corpus_folder):
    for fold in os.listdir(corpus_folder): 
        #Path to train and test sets. 
        train_path = corpus_folder + fold + '/corpus/train/'
        test_path = corpus_folder + fold + '/corpus/test/'
        val_path = corpus_folder + fold + '/corpus/validation/'

        for folder_path in [train_path, test_path, val_path]:
            for usr_file in os.listdir(folder_path): 
                file_path = folder_path + usr_file
                read_file = open(file_path, 'r')

                #File to eventually replace this with. 
                temp_file_name = usr_file + '.temp'
                temp_file = open(temp_file_name, 'w')
              
                print file_path

                for line in read_file:
                    phrase, sem_form, denotation, rec = line.split(';')
                    phrase = convert_numbers(phrase)
                    temp_file.write(phrase + ';' + sem_form + ';' + denotation + ';' + rec)

                read_file.close()
                temp_file.close()
                os.rename(temp_file_name, file_path)

def print_usage():
    print 'To populate result file with semantic forms: ./post_process.py populate_semantic_forms [result_file_name] [test_file_name]'
    print 'To add type constraints to semantic forms in a file: ./post_process.py type_constraints [file_name]'
    print 'Determine groundable items in corpus: ./post_process.py create_entities_for_grounding [corpus_folder] [dict_pickle_name]'
    print 'Convert numbers in train/test sets from alphabetical to numerical representation: ./post_process.py number_conversion [corpus_folder]'

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

    #Create pickled dictionaries containing the entities in a corpus. 
    elif sys.argv[1] == 'create_entities_for_grounding':
        if len(sys.argv) == 4:
            find_groundable_items(sys.argv[2], sys.argv[3])
        else:
            print_usage()

    elif sys.argv[1] == 'number_conversion': 
        if not len(sys.argv) == 3: 
            print_usage()
        else:
            convert_numbers_corpus(sys.argv[2])

    else:
        print_usage()
