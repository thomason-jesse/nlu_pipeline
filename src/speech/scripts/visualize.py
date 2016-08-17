#!/usr/bin/python

"""
The functions in this script may be used in
order to visualize results from experiments. 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import os
import sys
import matplotlib.pyplot as plt
import numpy as np

#More descriptive labels for experiment types. 
experiment_types = {'nbest': 'Sphinx ASR', 'cky': 'Parser Re-ranking'}

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
            if len(file_name.split('.')) == 2:
                #Name without extension should match given experiment_name.
                exp_name, exp_type = file_name.split('.') 

                if exp_name == experiment_name:
                    wer = extract_wer_from_file(wer_path + file_name)

                    #Creates dictionary if needed, then adds wer. 
                    if exp_type not in word_error_rates:
                        word_error_rates[exp_type] = []

                    word_error_rates[exp_type].append(wer)

    #Calculates averages for each experiment type and keeps track of its label.
    averages = []
    labels = []

    for experiment_type in word_error_rates:
        averages.append(sum(word_error_rates[experiment_type]) / len(word_error_rates[experiment_type]))
        labels.append(experiment_types[experiment_type])

    averages = tuple(averages)
    labels = tuple(labels)

    #Visualizes them using pyplot. 
    indeces = np.arange(len([element for element in averages]))
    width = 0.35

    fig, ax = plt.subplots()

    #Builds a bar for each average. 
    bars = ax.bar(indeces, averages, width, color = 'r')

    #Sets y range to 100.
    yticks = np.arange(0, 110, 10)
    ax.set_ylim([0, 100])
    ax.set_yticks(yticks)

    #Sets labels.
    ax.set_title('Avg. Word Error Rate on Test Set ' + experiment_name + ' Across 4 Folds')
    
    ax.set_xticks(indeces + (width / 2))
    ax.set_xticklabels(labels)

    ax.set_xlabel('Experiment Type')
    ax.set_ylabel('WER')

    plt.show()


"""
This function extracts the top n evaluation
results from a given file. 
"""
def extract_topn_result_from_file(topn_file_name):
    topn_file = open(topn_file_name, 'r')

    for line in topn_file:
        if line.startswith('ACCURACY'):
            return float(line.split(':')[1].strip()) * 100


"""
This function visualizes the top n evaluation results 
of different experiments by averaging their results across 
all of the folds of a cross fold validation experiment folder. 
"""
def visualize_cross_folds_top_n(folds_experiment_directory, experiment_name, n):
    #Keeps all data for each fold for each experiment type. 
    topn_results = {}

    for directory in os.listdir(folds_experiment_directory):
        #Path to WER evaluation files. 
        topn_path = folds_experiment_directory + '/' + directory + '/evaluations/top_' + str(n) + '/'

        #Looks for all result files from experiment. 
        for file_name in os.listdir(topn_path):
            if len(file_name.split('.')) == 2:
                #Name without extension should match given experiment_name.
                exp_name, exp_type = file_name.split('.') 

                if exp_name == experiment_name:
                    result = extract_topn_result_from_file(topn_path + file_name)

                    #Creates dictionary if needed, then adds wer. 
                    if exp_type not in topn_results:
                        topn_results[exp_type] = []

                    topn_results[exp_type].append(result)

    #Calculates averages for each experiment type and keeps track of its label.
    averages = []
    labels = []

    for experiment_type in topn_results:
        averages.append(sum(topn_results[experiment_type]) / len(topn_results[experiment_type]))
        labels.append(experiment_types[experiment_type])

    averages = tuple(averages)
    labels = tuple(labels)

    #Visualizes them using pyplot. 
    indeces = np.arange(len([element for element in averages]))
    width = 0.35

    fig, ax = plt.subplots()

    #Builds a bar for each average. 
    bars = ax.bar(indeces, averages, width, color = 'r')

    #Sets y range to 100.
    yticks = np.arange(0, 110, 10)
    ax.set_ylim([0, 100])
    ax.set_yticks(yticks)

    #Sets labels.
    ax.set_title('Avg. Percent of Phrases with Correct Hyp in Top ' + str(n) + ' Hypotheses for Test Set ' + experiment_name + ' Across 4 Folds')
    
    ax.set_xticks(indeces + (width / 2))
    ax.set_xticklabels(labels)

    ax.set_xlabel('Experiment Type')
    ax.set_ylabel('Percentage')

    plt.show()



"""
Prints the usage for the script. 
"""
def print_usage():
    print 'Visualize WER: ./visualize wer [folds_experiment_folder] [experiment_name]'
    print 'Visualize Top N results: ./visualize top_n [folds_experiment_folder] [experiment_name] [n]'

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'wer':
        if len(sys.argv) == 4:
            visualize_cross_folds_wer(sys.argv[2], sys.argv[3])
        else:
            print_usage()

    elif sys.argv[1] == 'top_n':
        if len(sys.argv) == 5:
            visualize_cross_folds_top_n(sys.argv[2], sys.argv[3], sys.argv[4])
        else:
            print_usage()

    else:
        print_usage()
