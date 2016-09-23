#!/usr/bin/python

import CKYParser
from utils import *
import sys
from heapq import heappush
from heapq import heappop

def get_list_key(item):
    return item[0]

def preprocess_line(line):
    phrase_raw, score = line.split('(')
    
    #Preprocesses phrase. 
    phrase = ''

    for word in phrase_raw.split():
        tokens = word.split("'")
        
        if len(tokens) == 2:
            phrase += tokens[0] + " '" + tokens[1] + ' '
        else:
            phrase += tokens[0] + ' '

    phrase = phrase.strip()

    #Preprocesses score. 
    score = score.strip(')\n')

    return [phrase, score]

def reorder_nbest_list(nbest_file_name, parser):
    nbest_file = open(nbest_file_name, 'r')
    reordered_file = open(nbest_file_name.split('.')[0] + '.reordered', 'w')

    #Keeps track of parses in case of repeats. 
    parse_scores = {}

    #List used to be re-ordered. 
    ordered_list = []

    #Keeps track of progress. 
    parse_count = 1

    for line in nbest_file:
        #Parses line to get phrase and score. 
        phrase, score = preprocess_line(line)

        #Checks if phrase has already been parsed before. 
        if phrase in parse_scores:
            most_likely_parse_score = parse_scores[phrase]
        else:
            most_likely_parse_score = parser.most_likely_cky_parse(phrase).next()[1]
            parse_scores[phrase] = most_likely_parse_score

        #Gets most likely parse score and pushes it onto heap mapped to its phrase. 
        ordered_list.append((most_likely_parse_score, score, phrase))

        #Updates progress. 
        sys.stdout.write('Parsed ' + str(parse_count) + ' phrases...\n')
        sys.stdout.flush()

        parse_count += 1

    #Writes re-ordered list to new file. 
    for parse in sorted(ordered_list, key=get_list_key, reverse=True):
        reordered_file.write(str(parse[0]) + ';' + str(parse[1]) + ';' + str(parse[2]) + '\n')

    #Closes files. 
    nbest_file.close()
    reordered_file.close()

def process_directory(directory, parser):
    for file_name in os.listdir(directory):
        if file_name.endswith('.nbest'):
            print '---------------------'
            print 'PARSING ' + file_name
            print '---------------------'

            reorder_nbest_list(directory + '/' + file_name, parser)

parser = load_obj_general(sys.argv[1])
process_directory(sys.argv[2], parser)
