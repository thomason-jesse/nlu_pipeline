#!/usr/bin/python

"""
This script contains functions for evaluating
experiment result files. 

Usage
-----
WER: ./evaluate.py wer [result_file] [evaluation_file_name]

    Parameters:
        
        result_file             - The file to evaluate. 

        evaluation_file_name    - The file in which to write the result
                                  of the evaluation to. 

Correct hypothesis in top n: ./evaluate.py top_n [result_file] [evaluation_file_name] [n]

    Parameters: 
        
        result_file             - The file to evaluate. 

        evaluation_file_name    - The file in which to write the result
                                  of the evaluation. 

        n                       - The number of hypotheses to consider for each phrase. 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import sys
import os
import subprocess

#Adds nlu_pipeline src folder in order to import modules from it. 
nlu_pipeline_path = '/home/rcorona/catkin_ws/src/nlu_pipeline/src/'
sys.path.append(nlu_pipeline_path)

#Nlu pipeline modules.
try:
    import CKYParser
    from utils import *
except ImportError:
    print 'ERROR: Unable to load nlu_pipeline_modules! Verify that nlu_pipeline_path is set correctly!'
    sys.exit()

#Stores the binary executables, some of which are used by this script. 
bin_path = os.path.abspath('../bin')

"""
This evaluates the WER rate
for all the phrases in a result
file using the top hypothesis. 
"""
def wer(result_file_name, evaluation_file_name):
    result_file = open(result_file_name, 'r')

    #Gets absolute path for evaluation file name. 
    evaluation_file_name = os.path.abspath(evaluation_file_name)

    #Reads in data. 
    data = []
    line = None

    while not line == '':
        line = result_file.readline()

        #Pound delimits phrase. 
        if line.startswith('#'):
            phrase = line.strip().split('#')[1].split(';')[0]

            #Line immediately after phrase is top scoring hypothesis. 
            line = result_file.readline().strip()
            hypothesis = line.split(';')[0]

            data.append([phrase, hypothesis])

    #Creates temporary files to run evaluations on.
    os.chdir('../bin')
    transcript_file = open('transcripts.txt', 'w')
    hyp_file = open('hyp.txt', 'w')

    #Used for naming phrases, needed by wer script. 
    count = 0

    for data_point in data:
        transcript_file.write(data_point[0] + ' (phrase' + str(count) + ')\n')
        hyp_file.write(data_point[1] + ' (phrase' + str(count) + ')\n')

        count += 1

    #Closes files so they may be read by WER evaluation script. 
    transcript_file.close()
    hyp_file.close()

    #Opens evaluation file for writing. 
    evaluation_file = open(evaluation_file_name, 'w')

    #Prepares arguments for running WER evaluation. 
    args = [bin_path + '/word_align.pl', 'transcripts.txt', 'hyp.txt']

    #Evaluates WER. 
    subprocess.call(args, stdout=evaluation_file, stderr=evaluation_file)

    #Deletes temp files and closes evaluation. 
    os.remove('transcripts.txt')
    os.remove('hyp.txt')
    evaluation_file.close()

def correct_in_top_n(result_file_name, evaluation_file_name, n):
    result_file = open(result_file_name, 'r')

    #Casts in case still in string form. 
    n = int(n)

    #Reads in data. 
    data = []
    line = None

    while not line == '':
        line = result_file.readline()

        #Pound delimits phrase. 
        if line.startswith('#'):
            phrase = line.strip().split('#')[1].split(';')[0]
            hypotheses = []


            #Reads in n top scoring hypotheses. 
            for i in range(n):
                line = result_file.readline().strip()

                hypothesis = line.split(';')[0]
                hypotheses.append(hypothesis)

            data.append([phrase, hypotheses])

    #Counters for evaluation metric. 
    total_phrases = len(data)
    num_correct = 0

    for data_point in data:
        phrase = data_point[0]
        hypotheses = data_point[1]
        correct_present = False

        #Looks for correct phrase in hypotheses. 
        for hypothesis in hypotheses:
            if hypothesis == phrase:
                correct_present = True

        if correct_present:
            num_correct += 1

    #Gets accuracy. 
    percent_correct = float(num_correct) / float(total_phrases)

    #Writes result of evaluations. 
    evaluation_file = open(evaluation_file_name, 'w')

    evaluation_file.write('NUM PHRASES: ' + str(total_phrases) + '\n')
    evaluation_file.write('NUM CORRECT WITHIN ' + str(n) + ' HYPOTHESES: ' + str(num_correct) + '\n')
    evaluation_file.write('ACCURACY: ' + str(percent_correct) + '\n')

    evaluation_file.close()

def semantic_form(result_file_name, evaluation_file_name, parser_file):
    result_file = open(result_file_name, 'r')

    #Gets absolute path for evaluation file name. 
    evaluation_file_name = os.path.abspath(evaluation_file_name)

    #Reads in data. 
    data = []
    line = None

    while not line == '':
        line = result_file.readline()

        #Pound delimits phrase. 
        if line.startswith('#'):
            phrase, semantic_form = line.strip().split('#')[1].split(';')

            #Line immediately after phrase is top scoring hypothesis. 
            line = result_file.readline().strip()
            hypothesis = line.split(';')[0]

            data.append([semantic_form, hypothesis, phrase])

    #Loads parser. 
    parser = load_obj_general(parser_file)

    #Counts for evaluation. 
    count_correct = 0
    total_count = 0

    #Evaluates data set. 
    for truth_hyp_phrase in data:
        true_semantic_form, hyp, phrase = truth_hyp_phrase
        true_semantic_form = ':'.join(true_semantic_form.split(':')[1:])

        #Gets semantic node from true semantic form. 
        true_node = parser.lexicon.read_semantic_form_from_str(true_semantic_form, None, None, [], False)
   
        #Parses top hypothesis to get semantic node. 
        hyp_node = parser.most_likely_cky_parse(hyp).next()[0].node

        #Evaluates equality. 
        if true_node == hyp_node:
            count_correct += 1

        total_count += 1

        print '**********************************************'
        print "Ground truth phrase: " + phrase
        print "Ground truth node: " + str(true_node) 
        print "Ground truth semantic form: " + parser.print_parse(true_node)
        print ''

        print "Hyp str: " + hyp 
        print "Hyp node: " + str(hyp_node)
        print "Hyp semantic form: " + parser.print_parse(hyp_node)
        print  ''

        print "Equal: " + str(true_node == hyp_node)
        print '***********************************************'
        print ''

def print_usage():
    print 'WER: ./evaluate.py wer [result_file] [evaluation_file_name]'
    print 'Correct hypothesis in top n: ./evaluate.py top_n [result_file] [evaluation_file_name] [n]'
    print 'Semantic form evaluation: ./evaluate.py semantic_form [result_file] [evaluation_file_name] [parser]'

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'wer':
        if not len(sys.argv) == 4:
            print_usage()
        else:
            wer(sys.argv[2], sys.argv[3])

    elif sys.argv[1] == 'top_n':
        if not len(sys.argv) == 5:
            print_usage()
        else:
            correct_in_top_n(sys.argv[2], sys.argv[3], sys.argv[4])

    elif sys.argv[1] == 'semantic_form':
        if not len(sys.argv) == 5:
            print_usage()
        else:
            semantic_form(sys.argv[2], sys.argv[3], sys.argv[4])

    else:
        print_usage()
