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
            phrase = line.strip().split('#')[1]

            #Line immediately after phrase is top scoring hypothesis. 
            line = result_file.readline().strip()
            hypothesis = line.split(';')[0]

            data.append([phrase, hypothesis])

    #Creates temporary files to run evaluations on.
    os.chdir('../bin')
    transcript_file = open('transcripts.txt', 'w')
    hyp_file = open('hyp.txt', 'w')

    for data_point in data:
        transcript_file.write(data_point[0] + '\n')
        hyp_file.write(data_point[1] + '\n')

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

def print_usage():
    print 'WER: ./evaluate.py wer [result_file] [evaluation_file_name]'

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'wer':
        if not len(sys.argv) == 4:
            print_usage()
        else:
            wer(sys.argv[2], sys.argv[3])

    else:
        print_usage()
