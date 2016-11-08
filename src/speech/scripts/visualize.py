#!/usr/bin/python

"""
The functions in this script may be used in
order to visualize results from experiments. 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import os
import sys

#Forces matplotlib to not use visual display. 
import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import analysis


#More descriptive labels for experiment types. 
experiment_types = {'nbest': 'Sphinx ASR', 'cky': 'Parser Re-ranking'}

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
        
        if experiment_type in experiment_types:
            labels.append(experiment_types[experiment_type])
        else:
            labels.append(experiment_type)

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
Plots the cdf of correct hypotheses indeces
in an experiment result file. 
"""
def visualize_correct_hyp_indeces(corpus_folder, exp_extension, cdf_file_name, percents_file_name):
    #Counts correct hyps and their indeces and generates cdf for it. 
    cdf, percent_increases = error_analysis.count_correct_hyp_indeces(corpus_folder, exp_extension)
    
    #First plots cdf. 
    plt.plot(cdf[0], cdf[1])
    plt.plot((10.0, 10.0), (0.0, 1.0))
    plt.xlim((0.0, 500.0))
    plt.savefig(cdf_file_name)
    plt.close()

    #Now percentage increases. 
    plt.xticks(np.arange(0.0, 100.0, 10.0))
    plt.plot(percent_increases[0], percent_increases[1])
    plt.savefig(percents_file_name)
    plt.close()


"""
Prints the usage for the script. 
"""
def print_usage():
    print 'Visualize WER: ./visualize wer [folds_experiment_folder] [experiment_name]'
    print 'Visualize Top N results: ./visualize top_n [folds_experiment_folder] [experiment_name] [n]'
    print 'Visualize Correct Hyp Indeces: ./visualize correct_hyp_indeces [folds_experiment_folder] [experiment ext] [cdf_file_name] [percent_increase_file_name]'

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

    elif sys.argv[1] == 'correct_hyp_indeces':
        if len(sys.argv) == 6:
            visualize_correct_hyp_indeces(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        else:
            print_usage()

    else:
        print_usage()
