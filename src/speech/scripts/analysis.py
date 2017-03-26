#!/usr/bin/python

"""
The functions in this script may
be used in order to conduct
analysis on experiment results and
evaluations (including error analysis). 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import sys
import os
from scipy.stats import ttest_rel
from experiments import tokenize_for_parser

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

    #Checks percentage of corpus <= 7 token length. 
    at_below_7 = 0.0
    num_phrases = 0.0

    #Iterates through each file in corpus folder. 
    for file_name in os.listdir(usr_file_folder):
        counter = 0

        #Counts lines and add phrase length. 
        for line in open(usr_file_folder + '/' + file_name, 'r'):
            phrase, _, _, _ = line.strip().split(';')

            #Gets number of tokens in phrase. 
            phrase_length = num_tokens(phrase)
            lengths.append(phrase_length)

            #Check length threshold. 
            if phrase_length <= 7:
                at_below_7 +=1.0

            num_phrases += 1.0

            counter += 1

        counts.append(counter)

        num_usrs += 1

    #Computes stats. 
    avg_length = float(sum(lengths)) / float(len(lengths))
    avg_num_phrases = float(sum(counts)) / float(len(counts))
    phrase_range = [min(counts), max(counts)]
    percent_at_below_7 = at_below_7 / num_phrases

    return [avg_length, avg_num_phrases, num_usrs, phrase_range, percent_at_below_7]

"""
Extracts recall, precision, and f1 scores
from a partial semantic form evaluation 
file. 
"""
def extract_sem_partial_scores_from_file(evaluation_file_name):
    evaluation_file = open(evaluation_file_name, 'r')
    lines = evaluation_file.readlines()

    precision = float(lines[0].strip().split(':')[1])
    recall = float(lines[1].strip().split(':')[1])
    f1 = float(lines[2].strip().split(':')[1])

    return [f1, precision, recall]

def extract_sem_full_avg_from_file(evaluation_file_name):
    evaluation_file = open(evaluation_file_name, 'r')
    lines = evaluation_file.readlines()

    total = float(lines[0].strip().split(':')[1])
    correct = float(lines[1].strip().split(':')[1])

    return correct / total

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
Consolidates the results from 
all folds on the top N evaluation
metric for an experiment type. 
"""
def consolidate_topn_results(fold_folder, experiment_name, n):
    #Scores for each fold. 
    folds = {}

    for fold_name in os.listdir(fold_folder):
        evaluation_file = fold_folder + '/' + fold_name + '/evaluations/top_' + str(n) + '/' + experiment_name 

        #Extracts scores from file.  
        score = extract_topn_result_from_file(evaluation_file)
    
        folds[fold_name] = score

    return folds


"""
Gathers the names of all experiments 
run by checking the current results files. 
"""
def get_experiment_names(fold_folder):
    #Every experiment folder should have a fold 0
    experiments_path = fold_folder + '/0/experiments/asr/result_files/'

    return os.listdir(experiments_path)

"""
Consolidates the results from 
all folds on the WER
evaluation metric for an experiment type. 
"""
def consolidate_wer_results(fold_folder, experiment_name):
    #Scores for each fold. 
    folds = {}

    for fold_name in os.listdir(fold_folder):
        evaluation_file = fold_folder + '/' + fold_name + '/evaluations/wer/' + experiment_name 

        #Extracts scores from file.  
        wer = extract_wer_from_file(evaluation_file)
    
        folds[fold_name] = wer

    return folds


"""
Consolidates the results from all
fold results on the partial semantic
form evaluation metric for an experiment type. 
"""
def consolidate_sem_partial_results(fold_folder, experiment_name):
    #Scores for each fold. 
    folds = {}

    for fold_name in os.listdir(fold_folder):
        evaluation_file = fold_folder + '/' + fold_name + '/evaluations/sem_partial/' + experiment_name 

        #Extracts scores from file.  
        f1, p, r = extract_sem_partial_scores_from_file(evaluation_file)
    
        folds[fold_name] = {'f1': f1, 'p': p, 'r': r}

    return folds

"""
Consolidates the results from all
fold results on the full semantic
form evaluation metric for an experiment type. 
"""
def consolidate_sem_full_results(fold_folder, experiment_name):
    results = {}

    for fold_name in os.listdir(fold_folder):
        evaluation_file = fold_folder + '/' + fold_name + '/evaluations/sem_full/' + experiment_name 

        #Extracts scores from file.  
        results[fold_name] = extract_sem_full_avg_from_file(evaluation_file)
  
    return results

"""
Gets the average value in a list
of floats. 
"""
def l_avg(l):
    return sum(l) / float(len(l))

"""
Performs a t-test between
two experiments accross all folds
for the full semantic evaluation metric.
"""
def t_test_sem_full(corpus_folds_dir, exp1, exp2):
    exp1_results = consolidate_sem_full_results(corpus_folds_dir, exp1)
    exp2_results = consolidate_sem_full_results(corpus_folds_dir, exp2)

    #Aligns scores for experiments. 
    exp1_scores = []
    exp2_scores = []

    #NOTE Assumes both experiments have same folds. 
    for fold_name in exp1_results:
        exp1_scores.append(exp1_results[fold_name])
        exp2_scores.append(exp2_results[fold_name])

    print exp1 + ' avg: ' + str(l_avg(exp1_scores))
    print exp2 + ' avg: ' + str(l_avg(exp2_scores))
    print 'p-value: ' + str(ttest_rel(exp1_scores, exp2_scores)[1])

"""
Performs a t-test between
two expeirments accross all folds
for the WER evaluation metric. 
"""
def t_test_wer(corpus_folds_dir, exp1, exp2):
    exp1_results = consolidate_wer_results(corpus_folds_dir, exp1)
    exp2_results = consolidate_wer_results(corpus_folds_dir, exp2)

    exp1_rates = []
    exp2_rates = []

    #NOTE Assumes both experiments have same folds. 
    for fold_name in exp1_results:
        exp1_rates.append(exp1_results[fold_name])
        exp2_rates.append(exp2_results[fold_name])

    print exp1 + ' avg: ' + str(l_avg(exp1_rates))
    print exp2 + ' avg: ' + str(l_avg(exp2_rates))
    print 'p-value: ' + str(ttest_rel(exp1_rates, exp2_rates)[1])

def t_test_topn(corpus_folds_dir, exp1, exp2, n):
    exp1_results = consolidate_topn_results(corpus_folds_dir, exp1, n)
    exp2_results = consolidate_topn_results(corpus_folds_dir, exp2, n)

    exp1_scores = []
    exp2_scores = []

    #NOTE Assumes both experiments have same folds. 
    for fold_name in exp1_results:
        exp1_scores.append(exp1_results[fold_name])
        exp2_scores.append(exp2_results[fold_name])

    print exp1 + ' avg: ' + str(l_avg(exp1_scores))
    print exp2 + ' avg: ' + str(l_avg(exp2_scores))
    print 'p-value: ' + str(ttest_rel(exp1_scores, exp2_scores)[1])

"""
Performs a t-test between 
two experiments accross all folds
for the partial semantic evaluation 
metric. 
"""
def t_test_sem_partial(corpus_folds_dir, exp1, exp2):
    exp1_results = consolidate_sem_partial_results(corpus_folds_dir, exp1)
    exp2_results = consolidate_sem_partial_results(corpus_folds_dir, exp2)

    #Prepares ordered arrays for to run t-test. 
    exp1_f1 = []
    exp2_f1 = []

    exp1_p = []
    exp2_p = []

    exp1_r = []
    exp2_r = []

    #NOTE Assumes both experiments have same folds, which should be the case if the script got to here. 
    for fold_name in exp1_results:
        exp1_f1.append(exp1_results[fold_name]['f1'])
        exp2_f1.append(exp2_results[fold_name]['f1'])

        exp1_p.append(exp1_results[fold_name]['p'])
        exp2_p.append(exp2_results[fold_name]['p'])

        exp1_r.append(exp1_results[fold_name]['r'])
        exp2_r.append(exp2_results[fold_name]['r'])

    print 'F1' 
    print '-----'
    print 'p-value: ' + str(ttest_rel(exp1_f1, exp2_f1)[1])
    print exp1 + ' avg.: ' + str(l_avg(exp1_f1)) 
    print exp2 + ' avg.: ' + str(l_avg(exp2_f1)) 
    print ''

    print 'P'
    print '-----'
    print 'p-value: ' + str(ttest_rel(exp1_p, exp2_p)[1])
    print exp1 + ' avg.: ' + str(l_avg(exp1_p))
    print exp2 + ' avg.: ' + str(l_avg(exp2_p))
    print ''

    print 'R'
    print '-----'
    print 'p-value: ' + str(ttest_rel(exp1_r, exp2_r)[1])
    print exp1 + ' avg.: ' + str(l_avg(exp1_r))
    print exp2 + ' avg.: ' + str(l_avg(exp2_r))

"""
Performs a t-test between the 
results of two experiments using
a specified evaluation metric. 
"""
def t_test(corpus_folds_dir, eval_metric, exp1, exp2):
    if eval_metric == 'sem_partial':
        t_test_sem_partial(corpus_folds_dir, exp1, exp2)

    elif eval_metric == 'sem_full':
        t_test_sem_full(corpus_folds_dir, exp1, exp2)

    elif eval_metric == 'wer':
        t_test_wer(corpus_folds_dir, exp1, exp2)

    elif eval_metric == 'top_1':
        t_test_topn(corpus_folds_dir, exp1, exp2, 1)

    elif eval_metric == 'top_5':
        t_test_topn(corpus_folds_dir, exp1, exp2, 5)

    else:
        print 'Invalid evaluation metric specified!'
        sys.exit()

"""
Performs t-test for tuning results. 
"""
def t_test_tuning(exp_file_name, weight1, weight2):
    exp_file = open(exp_file_name, 'r')

    for line in exp_file:
        weight = line.split(':')[0] 
        
        if weight == weight1:
            exp1_values = [float(element) for element in line.split(':')[1].strip().split(';')]
        elif weight == weight2:
            exp2_values = [float(element) for element in line.split(':')[1].strip().split(';')]

    print 'p-value: ' + str(ttest_rel(exp1_values, exp2_values)[1])
    print str(weight1) + ': ' + str(float(sum(exp1_values)) / float(len(exp1_values)))
    print str(weight2) + ': ' + str(float(sum(exp2_values)) / float(len(exp2_values)))

"""
Averages number of training phrases
over folds in an experiment folder. 
"""
def compute_avg_training_phrases(corpus_folder_path, filter_len=None): 
    avg_training_examples = []
    avg_test_examples = []

    for fold in os.listdir(corpus_folder_path):
        training_folder = corpus_folder_path + fold + '/corpus/train/'
        test_folder = corpus_folder_path + fold + '/corpus/test/'

        #Now count number of lines (i.e. training examples) per file. 
        num_training_examples = 0
        num_test_examples = 0

        for usr_file in os.listdir(training_folder):
            lines = [line for line in open(training_folder + usr_file, 'r')]

            #Filter phrase length if desired.
            if not filter_len == None: 
                lines = [line for line in lines if len(tokenize_for_parser(line.split(';')[0]).split()) <= filter_len]

            num_training_examples += len(lines)

        avg_training_examples.append(num_training_examples)

        #Do the same for test examples. 
        for usr_file in os.listdir(test_folder):
            lines = [line for line in open(test_folder + usr_file, 'r')]

            #Filter phrase length if desired.
            if not filter_len == None: 
                lines = [line for line in lines if len(tokenize_for_parser(line.split(';')[0]).split()) <= filter_len]

            num_test_examples += len(lines)

        avg_test_examples.append(num_test_examples)

    #Now compute mean and display. 
    mean_train = float(sum(avg_training_examples)) / float(len(avg_training_examples))
    mean_test = float(sum(avg_test_examples)) / float(len(avg_test_examples))

    print 'Average number of training examples in corpus: ' + str(mean_train)
    print 'Average number of test examples in corpus: ' + str(mean_test)

"""
Prints the usage options
for this script. 
"""
def print_usage():
    print 'Print parser parameters: ./analysis.py print_parser [parser_file]'
    print 'Count correct hypothesis indeces: ./analysis.py correct_hyp_indeces [corpus_folder] [exp_extension]'
    print 'Gather corpus statistics: ./analysis.py corpus_stats [corpus_usr_file_folder]'
    print 't-test for statistical significance in diff between two experiment types: ./analysis.py t_test [corpus_folds_dir] [evaluation_metric] [exp1_name] [exp2_name]'
    print 't-test for tuning results: ./analysis.py t_test_tuning [results_file_name] [weight1] [weight2]'
    print 'Average number of training/test phrases for an experiment folder: ./analysis.py avg_num_data_points [corpus_folder] [Optional: filter_length]'

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

    elif sys.argv[1] == 't_test_tuning':
        if len(sys.argv) == 5:
            t_test_tuning(sys.argv[2], sys.argv[3], sys.argv[4])
        else:
            print_usage()

    elif sys.argv[1] == 'corpus_stats':
        if len(sys.argv) == 3:
            avg_len, avg_phrase, num_usrs, phrase_range, at_below_7 = extract_corpus_statistics(sys.argv[2])

            print "AVERAGE TOKENS PER PHRASE: " + str(avg_len)
            print "AVERAGE PHRASES PER USER: " + str(avg_phrase)
            print "NUM PHRASE RANGE: " + str(phrase_range)
            print "NUMBER OF USERS: " + str(num_usrs)
            print "Percent of phrases at or below 7 tokens: " + str(at_below_7)

        else:
            print_usage()

    elif sys.argv[1] == 't_test':
        if len(sys.argv) == 6:
            t_test(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        else:
            print_usage()

    elif sys.argv[1] == 'avg_num_data_points':
        if len(sys.argv) == 3:
            compute_avg_training_phrases(sys.argv[2])
        elif len(sys.argv) == 4:
            compute_avg_training_phrases(sys.argv[2], int(sys.argv[3]))
        else:
            print_usage()

    else:
        print_usage()
