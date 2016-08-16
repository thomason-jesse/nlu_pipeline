#!/usr/bin/python

"""
The functions in this script may be used in
order to visualize results from experiments. 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import os
import sys

"""
This function extracts the WER rate 
reported by the word_align.pl script
in a given file. 
"""
def extract_wer_from_file(wer_file_name):
    wer_file = open(wer_file_name)

    #Line with WER follows distinct format. 
    for line in wer_file:
        if line.startswith('TOTAL Percent'):
            wer = float(line.split('Error = ')[1].split('%')[0])

            return wer


"""
This function visualizes the WER of different experiments by 
averaging their results across all of the folds of a cross
fold validation experiment folder. 
"""
def visualize_cross_folds_wer(folds_experiment_directory, experiment_name):
    #Keeps all data for each fold for each experiment type. 
    word_error_rates = {}

    for directory in os.listdir(folds_experiment_directory):
        #Path to WER evaluation files. 
        wer_path = folds_experiment_directory + '/' + directory + '/evaluations/wer/'

        #Looks for all result files from experiment. 
        for file_name in os.listdir(wer_path): 
            #Name without extension should match given experiment_name.
            exp_name, exp_type = file_name.split('.') 

            if exp_name == experiment_name:
                wer = extract_wer_from_file(wer_path + file_name)

                #Creates dictionary if needed, then adds wer. 
                if exp_type not in word_error_rates:
                    word_error_rates[exp_type] = []

                word_error_rates[exp_type].append(wer)

    print word_error_rates

    sys.exit()

"""
Prints the usage for the script. 
"""
def print_usage():
    print  'Visualize WER: ./visualize wer [folds_experiment_folder] [experiment_name]'

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'wer':
        if len(sys.argv) == 4:
            visualize_cross_folds_wer(sys.argv[2], sys.argv[3])
        else:
            print_usage()

    else:
        print_usage()
