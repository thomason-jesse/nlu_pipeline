#!/usr/bin/python

import sys

"""
Creates a training file for a parser
in the desired folder
using all the corpus files found in the 
given corpus_folder. 
"""
def create_parser_training_file(ontology_filepath, parser_training_filepath): 
    ontology_file = open(ontology_filepath, 'r')
    parser_training_file = open(parser_training_filepath, 'w')

    person_list = []
    location_list = []
	
    for line in ontology_file:
        if line.endswith(':p\n'):
            person = line.split(':')[0]
            person_list.append(person)
        if line.endswith(':l\n'):
            location = line.split(':')[0]
            #if(location.startswith('l3_')):
            #    location_list.append(location[1:])
            #else:
            location_list.append(location)
    ontology_file.close()

    for person in person_list:
        utterance = 'walk to ' + person + ' \'s office'
        semantic_form = 'M : walk(the(lambda x:l.(and(office(x), possesses(x, ' + person + ')))))'
        parser_training_file.write(utterance + '\n')
        parser_training_file.write(semantic_form + '\n\n')

    for location in location_list:
        utterance = 'walk to ' + location
        semantic_form = 'M : walk(' + location + ')'
        parser_training_file.write(utterance + '\n')
        parser_training_file.write(semantic_form + '\n\n')

    #Gets phrase length limit.
    #if filter_len == None:
    #    length_lim = float('inf')
    #else:
    #    length_lim = int(filter_len)

    #Consolidates all corpus data into one file for training. 
    #for file_name in os.listdir(corpus_folder):
    #    if file_name.endswith('.usr'):
    #        corpus_file = open(corpus_folder + '/' + file_name, 'r')

            #Adds a each training pair in necessary format for parser training. 
    #        for line in corpus_file:
    #            phrase, semantic_form, _, _ = line.split(';')

                #Adds phrase if it is under the token length limit. 
    #            if get_tokenized_phrase_len(phrase) <= length_lim:
    #                parser_training_file.write(get_tokenized_phrase(phrase) + '\n' + semantic_form + '\n\n')

    #Closes the training file. 
    parser_training_file.close()

create_parser_training_file(sys.argv[1], sys.argv[2])
