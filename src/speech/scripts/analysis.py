#!/usr/bin/python

"""
The functions in this script may
be used in order to conduct error
analysis on experiment results and
evaluations. 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import sys
import os

#Path to nlu_pipeline modules. #TODO Change if needed. 
nlu_pipeline_path = '/scratch/cluster/rcorona/nlu_pipeline/src/'
sys.path.append(nlu_pipeline_path)

#Nlu pipeline modules.
try:
    import CKYParser
    from utils import *
except ImportError:
    print 'ERROR: Unable to load nlu_pipeline_modules! Verify that nlu_pipeline_path is set correctly!'
    sys.exit()

"""
This method will go through all of the top N
hypothesis files for all folds in a corpus
and count what hypothesis index (if any) the
correct hypothesis was at. 
"""
def count_correct_hyp_indeces(corpus_folder, exp_extension):
    #Keeps counts of correct hypothesis indeces.  
    counts = {}
    num_phrases = 0
    
    #Iterates through each fold in folder. 
    for fold in os.listdir(corpus_folder):
        
        #Iterate through experiment results folder. 
        for result_file_name in os.listdir(corpus_folder + '/' + fold + '/experiments/asr/result_files'):
            
            #Only gets files with the correct experiment extension. 
            if '.' + exp_extension in result_file_name:

                #Gets experiment name creates hyp count list for this file. 
                experiment = result_file_name.split('.')[0]

                #Counts correct hypothesis in file.
                result_file = open(corpus_folder + '/' + fold + '/' + '/experiments/asr/result_files/' + result_file_name, 'r')
                
                line = result_file.readline()

                while not line == '':
                    # '#' denotes start of new phrase hypothesis list. 
                    if line.startswith('#'): 
                        #Get correct phrase and start new hyp list. 
                        ground_truth = line.split('#')[1].strip()
                        count = 0
                        lowest_correct_index = float('inf')
                        num_phrases += 1
                    else:
                        #Get hypothesis and compare to ground truth. 
                        hyp = line.split(';')[0]

                        if hyp == ground_truth and count <= lowest_correct_index:
                            #Add count to count map and set lowest correct index to this one. 
                            lowest_correct_index = count

                            if count in counts:
                                counts[count] += 1.0
                            else:
                                counts[count] = 1.0

                    #Increments count and gets next line. 
                    count += 1
                    line = result_file.readline()

    #Gets total number of recoverable correct hypotheses. 
    total = 0.0

    for index in counts:
        total += counts[index]

    #Now generates CDF and percentage increases. 
    summation = 0.0
    percent_buff = None
    cdf = [[], []]
    percentage_increases = [[], []]

    for index in sorted(counts.keys()):
        summation += counts[index]
        percent = summation / total

        cdf[0].append(index)
        cdf[1].append(percent)

        #TODO Make this limit dynamic (i.e. add a variable). 
        if (index <= 100):
            #Skips first index. 
            if percent_buff:
                percentage_increases[0].append(index)
                percentage_increases[1].append(percent - percent_buff)

            percent_buff = percent

    #Returns result of analysis. 
    return [cdf, percentage_increases]

        

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
Tokenizes phrase and returns the number
of tokens within it. 
"""
def num_tokens(phrase):
    return len(phrase.replace("'s", " 's").split())

"""
Used to extract statistics about a corpus. 
"""
def extract_corpus_statistics(usr_file_folder):
    #Used to compute average phrase length. 
    lengths = []

    #Used to get average number of phrases per user. 
    counts = []

    #Keeps track of how many user files there are. 
    num_usrs = 0

    #Iterates through each file in corpus folder. 
    for file_name in os.listdir(usr_file_folder):
        counter = 0

        #Counts lines and add phrase length. 
        for line in open(usr_file_folder + '/' + file_name, 'r'):
            phrase, _, _, _ = line.strip().split(';')

            #Gets number of tokens in phrase. 
            lengths.append(num_tokens(phrase))

            counter += 1

        counts.append(counter)

        num_usrs += 1

    #Computes stats. 
    avg_length = float(sum(lengths)) / float(len(lengths))
    avg_num_phrases = float(sum(counts)) / float(len(counts))
    phrase_range = [min(counts), max(counts)]

    return [avg_length, avg_num_phrases, num_usrs, phrase_range]

"""
Prints the usage options
for this script. 
"""
def print_usage():
    print 'Print parser parameters: ./analysis.py print_parser [parser_file]'
    print 'Count correct hypothesis indeces: ./analysis.py correct_hyp_indeces [corpus_folder] [exp_extension]'
    print 'Gather corpus statistics: ./analysis.py corpus_stats [corpus_usr_file_folder]'


if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'print_parser':
        if len(sys.argv) == 3:
            dump_cky_parser_parameters(sys.argv[2])
        else:
            print_usage()
    elif sys.argv[1] == 'correct_hyp_indeces':
        if len(sys.argv) == 4:
            cdf, increases = count_correct_hyp_indeces(sys.argv[2], sys.argv[3])
            print cdf

        else:
            print_usage()

    elif sys.argv[1] == 'corpus_stats':
        if len(sys.argv) == 3:
            avg_len, avg_phrase, num_usrs, phrase_range = extract_corpus_statistics(sys.argv[2])

            print "AVERAGE TOKENS PER PHRASE: " + str(avg_len)
            print "AVERAGE PHRASES PER USER: " + str(avg_phrase)
            print "NUM PHRASE RANGE: " + str(phrase_range)
            print "NUMBER OF USERS: " + str(num_usrs)

        else:
            print_usage()

    else:
        print_usage()
