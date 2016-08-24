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
#os.remove('transcripts.txt')
#os.remove('hyp.txt')
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

def print_usage():
    print 'WER: ./evaluate.py wer [result_file] [evaluation_file_name]'
    print 'Correct hypothesis in top n: ./evaluate.py top_n [result_file] [evaluation_file_name] [n]'

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

    else:
        print_usage()
