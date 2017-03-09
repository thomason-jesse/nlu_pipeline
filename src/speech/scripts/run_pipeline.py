#!/usr/bin/python

"""
This script may be used to run the entire
pipeline in as few steps as possible. 


Author: Rodolfo Corona, rcorona@utexas.edu
"""

#Python imports
import matplotlib
matplotlib.use('Agg')

import sys
import os
import subprocess
import numpy as np
import matplotlib.pyplot as plt    

#Speech pipeline imports.
import preprocess
import train
import evaluate
import analysis

"""
Sets up an experiment
folder with a desired number
of folds. Automatically creates
necessary training files to 
train the pipeline's models.
"""
def set_up(preprocessed_corpus_path, n_fold_dir_path, n):
    #Sets up experiment folder structure. 
    print 'Setting up experiment directory structure...'
    preprocess.split_into_folds(preprocessed_corpus_path, n_fold_dir_path, int(n))

    #Creates training files for parser and ASR in each fold.
    print 'Preprocessing folds and creating training files...'

    for fold_dir in os.listdir(n_fold_dir_path):
        fold_path = n_fold_dir_path + '/' + fold_dir

        #Paths to fold's training and test sets. 
        train_path = fold_path + '/corpus/train/'
        test_path = fold_path + '/corpus/test/'

        #Adds type constraints to file's semantic forms. 
        for usr_file in os.listdir(train_path):
            preprocess.add_type_constraints_to_usr_file(train_path + usr_file)

        for usr_file in os.listdir(test_path):
            preprocess.add_type_constraints_to_usr_file(test_path + usr_file)

        #Parser training file.
        preprocess.create_parser_training_file(train_path, fold_path + '/parser_training/')

        #ASR training files.
        preprocess.create_asr_transcripts(train_path, fold_path + '/asr_training/')
        preprocess.create_lm_training_file(train_path, fold_path + '/asr_training/')

        #Creates files for testing. 
        preprocess.create_asr_test_files(test_path, fold_path + '/experiments/asr/test_files/', 2)

"""
Trains language models, acoustic models, and
parsers for each fold in an experiment directory. 
"""
def train_models(experiment_dir_path): 
    for fold_name in os.listdir(experiment_dir_path):
        #Forks and nohups training operations. 
        pid = os.fork()

        if pid == 0:
            fold_path = experiment_dir_path + '/' + fold_name

            #Trains ASR LM.
            #Prepares arguments. 
            args = ['./train.py', 'new_lm']                         #Specifies LM training. 
            args.append(fold_path + '/asr_training/lm_train.txt')   #LM training file. 
            args.append(fold_path + '/models/in-domain.lm')         #LM model save path. 

            #Log file for output.
            log_file = open(fold_path + '/logs/training/train_lm.txt', 'w')
           
            #Calls LM training program. 
            subprocess.call(args, stdout=log_file, stderr=log_file)
            log_file.close()
           
            #Adapts acoustic model to domain.
            args = ['./train.py', 'adapt_ac_model']                     #Specifies acoustic model adaptation. 
            args.append('../resources/en-us/')                          #The directory containing the original acoustic model to be adapted. 
            args.append(fold_path)                                      #The fold path. 
            args.append('../corpora/raw/headset/')                      #The directory where the raw recording files are kept. 
            args.append('../resources/cmudict-en-us.dict')              #The dictionary (i.e. lexicon) used by the ASR system. 

            #Log file. 
            log_file = open(fold_path + '/logs/training/adapt_ac.txt', 'w')

            #Calls AC model adaptation program. 
            subprocess.call(args, stdout=log_file, stderr = log_file)
            log_file.close()

            #Trains parser.
            args = ['./train.py', 'new_parser']                             #Specifies parser training. 
            args.append(fold_path + '/parser_training/parser_train.txt')    #Parser training file. 
            args.append(fold_path + '/models/parser.cky')                   #Parser pickle file to save model. 
            args.append('../resources/ont.txt')                             #The ontology to be used. 
            args.append('../resources/lex.txt')                             #The lexicon to be used. 
            args.append('50.0')                                             #The lexicon weight to use. 

            #Log file. 
            log_file = open(fold_path + '/logs/training/train_parser.txt', 'w')

            #Calls parser training. 
            subprocess.call(args, stdout=log_file, stderr=log_file)
            log_file.close()

            #Exits child process. 
            sys.exit()

"""
Runs the first experiment, which consists
of running the ASR on the corpus' first fold, ending with
re-ranking it using the CKY-reranker. Also re-ranks the second
test set in order to be able to compare performance with second experiment
(re-ranking and re-training). 
"""
def exp1(experiment_dir_path): 
    #Runs a separate thread for each fold. 
    for fold_name in os.listdir(experiment_dir_path):
        pid = os.fork()

        if pid == 0: 
            fold_path = experiment_dir_path + '/' + fold_name
            result_file_path = fold_path + '/experiments/asr/result_files/'
            test_file_path = fold_path + '/experiments/asr/test_files/'

            #Runs ASR and parser re-ranking on each test fold.
            pids = []

            for test_file in os.listdir(test_file_path):   
                #Speed up by forking. 
                pid = os.fork()

                #Add pid to list of pids to wait for. 
                if not pid == 0:
                    pids.append(pid)
                #Child otherwise, so execute experiment. 
                else:
                    test_name = test_file.split('.')[0]

                    args = ['./experiments.py', 'asr_n_best']               #Specifies ASR experiment. 
                    args.append('../sphinx/build/libsphinx.so')             #Path to shared library file used by Pocketsphinx. 
                    args.append(fold_path + '/models/adapted_ac_model/')    #Path to fold's acoustic model. 
                    args.append(fold_path + '/models/in-domain.lm')         #Path to fold's in-domain LM. 
                    args.append('../resources/cmudict-en-us.dict')          #Path to english dictionary used by Sphinx. 
                    args.append(test_file_path + test_file)                 #Path to the test file being experimented on. 
                    args.append(result_file_path + test_name + '.nbest')    #Path to result file from experiment. 
                    args.append('10')                                       #Specifies that top 10 hypotheses should be used. 

                    #Log file. 
                    log_file = open(fold_path + '/logs/experiments/' + test_name + '.nbest', 'w')

                    #Runs experiment. 
                    subprocess.call(args, stdout=log_file, stderr=log_file)
                    
                    #Re-ranks file using CKY Parser. 
                    args = ['./experiments.py', 'parser_rerank'] 
                    args.append(result_file_path + test_name + '.nbest') 
                    args.append(result_file_path + test_name + '.cky') 
                    args.append(fold_path + '/models/parser.cky')
                    args.append(fold_name + '_' + test_name)

                    #Log file. 
                    log_file = open(fold_path + '/logs/experiments/' + test_name + '.cky', 'w')

                    #Runs  experiment. 
                    subprocess.call(args, stdout=log_file, stderr=log_file)

                    sys.exit()

            #Wait for all test files to be experimented on before evaluating. 
            for pid in pids:
                os.waitpid(pid)

            #Now evaluates files. 
            for result_file in os.listdir(result_file_path):
                result_file_name = result_file_path + result_file
                eval_path = fold_path + '/evaluations'
                parser_path = fold_path + '/models/parser.cky'

                evaluate.semantic_form(result_file_name, eval_path + '/sem_full/' + result_file, parser_path)
                evaluate.semantic_form_partial(result_file_name, eval_path + '/sem_partial/' + result_file, parser_path)
                evaluate.wer(result_file_name, eval_path + '/wer/' + result_file)
                evaluate.correct_in_top_n(result_file_name, eval_path + '/top_1/' + result_file, 1)
                evaluate.correct_in_top_n(result_file_name, eval_path + '/top_5/' + result_file, 5)

            #Child process exit. 
            sys.exit()

"""
Runs the second experiment, which consists
of re-training (i.e. adapting) the acoustic model
using the results on the first test set (from the
first experiment) as the transcription data. 
"""
def exp2(experiment_dir_path):
    #Runs a separate thread for each fold. 
    for fold_name in os.listdir(experiment_dir_path):
        pid = os.fork()

        if pid == 0: 
            fold_path = experiment_dir_path.strip('/') + '/' + fold_name 
            result_file_path = fold_path + '/experiments/asr/result_files/'
            test_file_path = fold_path + '/experiments/asr/test_files/'
            asr_path = fold_path + '/asr_training/'

            #Create ASR retraining file for both baseline and re-ranked result files. 
            preprocess.extract_transcript_from_results_file(result_file_path + '0.nbest',
                                                            asr_path + 're_train_baseline.fileids', 
                                                            asr_path + 're_train_baseline.transcription',     
                                                            test_file_path + '0.test')

            preprocess.extract_transcript_from_results_file(result_file_path + '0.cky',
                                                            asr_path + 're_train_cky.fileids',
                                                            asr_path + 're_train_cky.transcription',
                                                            test_file_path + '0.test')

            #Now adapts acoustic models (i.e. re-training). 
            args = ['./train.py', 'adapt_ac_model']                     #Specifies acoustic model adaptation. 
            args.append(fold_path + '/models/adapted_ac_model/')        #The directory containing the original acoustic model to be adapted. 
            args.append(fold_path)                                      #The fold path. 
            args.append('../corpora/raw/headset/')                      #The directory where the raw recording files are kept. 
            args.append('../resources/cmudict-en-us.dict')              #The dictionary (i.e. lexicon) used by the ASR system.
            args.append('re_trained_baseline_ac_model')                 #Name of directory for new ac model. 
            args.append('re_train_baseline')                            #Prefix for asr training files. 

            #Log file. 
            log_file = open(fold_path + '/logs/training/adapt_ac_baseline.txt', 'w')

            #Calls AC model adaptation program. 
            subprocess.call(args, stdout=log_file, stderr = log_file)
            log_file.close()

            args = ['./train.py', 'adapt_ac_model']                     #Specifies acoustic model adaptation. 
            args.append(fold_path + '/models/adapted_ac_model/')        #The directory containing the original acoustic model to be adapted. 
            args.append(fold_path)                                      #The fold path. 
            args.append('../corpora/raw/headset/')                      #The directory where the raw recording files are kept. 
            args.append('../resources/cmudict-en-us.dict')              #The dictionary (i.e. lexicon) used by the ASR system.
            args.append('re_trained_cky_ac_model')                      #Name of directory for new ac model. 
            args.append('re_train_cky')                                 #Prefix for asr training files. 

            #Log file. 
            log_file = open(fold_path + '/logs/training/adapt_ac_cky.txt', 'w')

            #Calls AC model adaptation program. 
            subprocess.call(args, stdout=log_file, stderr = log_file)
            log_file.close()

            #Runs ASR and parser re-ranking for each re-trained model.
            for model in ['baseline', 'cky']:    
                ac_model = 're_trained_' + model + '_ac_model'

                args = ['./experiments.py', 'asr_n_best']                                   #Specifies ASR experiment. 
                args.append('../sphinx/build/libsphinx.so')                                 #Path to shared library file used by Pocketsphinx. 
                args.append(fold_path + '/models/' + ac_model)                              #Path to fold's acoustic model. 
                args.append(fold_path + '/models/in-domain.lm')                             #Path to fold's in-domain LM. 
                args.append('../resources/cmudict-en-us.dict')                              #Path to english dictionary used by Sphinx. 
                args.append(test_file_path + '1.test')                                      #Path to the test file being experimented on. 
                args.append(result_file_path + '1.nbest_re_trained_' + model)               #Path to result file from experiment. 
                args.append('10')                                                           #Specifies that top 10 hypotheses should be used. 

                #Log file. 
                log_file = open(fold_path + '/logs/experiments/1.nbest_re_trained_' + model, 'w')

                #Runs experiment. 
                subprocess.call(args, stdout=log_file, stderr=log_file)
                
                #Re-ranks file using CKY Parser. 
                args = ['./experiments.py', 'parser_rerank']                    #Specifies that parser re-ranking should be performed. 
                args.append(result_file_path + '1.nbest_re_trained_' + model)   #Path to file to re-rank. 
                args.append(result_file_path + '1.cky_re_trained_' + model)     #Path to re-ranked file. 
                args.append(fold_path + '/models/parser.cky')                   #Path to parser to use for re-ranking. 
                args.append(fold_name)                                          #Path to the experiment fold. 

                #Log file. 
                log_file = open(fold_path + '/logs/experiments/1.cky_re_trained_' + model, 'w')

                #Runs  experiment. 
                subprocess.call(args, stdout=log_file, stderr=log_file)
      
            #Now evaluates files. 
            for model in ['baseline', 'cky']:
                for exp in ['cky', 'nbest']:
                    result_file = '1.' + exp + '_re_trained_' + model 
                    result_file_name = result_file_path + result_file
                    eval_path = fold_path + '/evaluations'
                    parser_path = fold_path + '/models/parser.cky'

                    evaluate.semantic_form(result_file_name, eval_path + '/sem_full/' + result_file, parser_path)
                    evaluate.semantic_form_partial(result_file_name, eval_path + '/sem_partial/' + result_file, parser_path)
                    evaluate.wer(result_file_name, eval_path + '/wer/' + result_file)
                    evaluate.correct_in_top_n(result_file_name, eval_path + '/top_1/' + result_file, 1)
                    evaluate.correct_in_top_n(result_file_name, eval_path + '/top_5/' + result_file, 5)

            sys.exit()

"""
A simple averaging function. 
"""
def average(l):
    return float(sum(l)) / float(len(l))


"""
Consolidates and averages results for every evaluation
metric over all folds into pyplot graphs. 
"""
def consolidate_results_to_pyplot(experiment_dir_path, consolidated_files_dir):
    #Retrieves all experiment names in order to gather results. 
    experiment_names = analysis.get_experiment_names(experiment_dir_path)
    
    #Results will be stored by experiment. 
    results = {exp: {} for exp in experiment_names}

    for exp in results:
        results[exp] = {'wer': None, 'r': None, 'p': None, 'f1': None, 'sem_full': None, 'top_1': None, 'top_5': None}   

    for exp in experiment_names: 
        results[exp]['wer'] = average(analysis.consolidate_wer_results(experiment_dir_path, exp).values())
        results[exp]['sem_full'] = average(analysis.consolidate_sem_full_results(experiment_dir_path, exp).values())
        results[exp]['top_1'] = average(analysis.consolidate_topn_results(experiment_dir_path, exp, 1).values())
        results[exp]['top_5'] = average(analysis.consolidate_topn_results(experiment_dir_path, exp, 5).values())

        #Processes precision, recall, and f1 scores further. 
        sem_partial = analysis.consolidate_sem_partial_results(experiment_dir_path, exp).values()
        results[exp]['r'] = average([scores['r'] for scores in sem_partial])
        results[exp]['p'] = average([scores['p'] for scores in sem_partial])
        results[exp]['f1'] = average([scores['f1'] for scores in sem_partial])

    #Cleaner names for experimenents for tables. 
    exp_names_0 = {'0.nbest': 'BN',
                 '0.cky': 'RN'}

    exp_names_1 = {'1.nbest': 'BN',
                 '1.cky': 'RN',
                 '1.nbest_re_trained_baseline': 'BB',
                 '1.nbest_re_trained_cky': 'BR',
                 '1.cky_re_trained_baseline': 'RB',
                 '1.cky_re_trained_cky': 'RR'}

    metric_names = {"wer": "WER", "r": "Recall", "p": "Precision", "f1": "F1", "sem_full": "Full Semantic Form",
                    "top_1": "Top 1", "top_5": "Top 5"}

    #Make plots for each metric.
    for metric in ['wer', 'r', 'p', 'f1', 'sem_full', 'top_1', 'top_5']:
        for plt_group in [exp_names_0, exp_names_1]:
            num_groups = len(plt_group.keys())    
            index = np.arange(num_groups)
            bar_width = 0.35

            scores = [results[exp][metric] for exp in plt_group.keys()]

            plt.bar(index, scores, bar_width, color='r')
            plt.xlabel('Conditions')
            plt.ylabel('Average')
            plt.title(metric_names[metric] + ' Average Scores')
            plt.xticks(index + bar_width / 2.0, [plt_group[exp] for exp in plt_group.keys()])

            plt.savefig(metric + '_' + str(plt_group.keys()[0].split('.')[0]) + '.png')
            plt.close()

"""
Consolidates and averages results for every evaluation
metric over all folds into LaTex table format.  
"""
def consolidate_results_to_latex(experiment_dir_path, consolidated_files_dir):
    #Retrieves all experiment names in order to gather results. 
    experiment_names = analysis.get_experiment_names(experiment_dir_path)
    
    #Results will be stored by experiment. 
    results = {exp: {} for exp in experiment_names}

    for exp in results:
        results[exp] = {'wer': None, 'sem_partial': {}, 'sem_full': None, 'top_1': None, 'top_5': None}   

    for exp in experiment_names: 
        results[exp]['wer'] = average(analysis.consolidate_wer_results(experiment_dir_path, exp).values())
        results[exp]['sem_full'] = average(analysis.consolidate_sem_full_results(experiment_dir_path, exp).values())
        results[exp]['top_1'] = average(analysis.consolidate_topn_results(experiment_dir_path, exp, 1).values())
        results[exp]['top_5'] = average(analysis.consolidate_topn_results(experiment_dir_path, exp, 5).values())

        #Processes precision, recall, and f1 scores further. 
        sem_partial = analysis.consolidate_sem_partial_results(experiment_dir_path, exp).values()
        results[exp]['sem_partial']['r'] = average([scores['r'] for scores in sem_partial])
        results[exp]['sem_partial']['p'] = average([scores['p'] for scores in sem_partial])
        results[exp]['sem_partial']['f1'] = average([scores['f1'] for scores in sem_partial])

    #Writes results to files.
    wer_file = open(consolidated_files_dir + '/wer.txt', 'w') 
    top_1_file = open(consolidated_files_dir + '/top_1.txt', 'w') 
    top_5_file = open(consolidated_files_dir + '/top_5.txt', 'w') 
    sem_partial_file_f1 = open(consolidated_files_dir + '/sem_partial_f1.txt', 'w') 
    sem_partial_file_p = open(consolidated_files_dir + '/sem_partial_p.txt', 'w') 
    sem_partial_file_r = open(consolidated_files_dir + '/sem_partial_r.txt', 'w') 
    sem_full_file = open(consolidated_files_dir + '/sem_full.txt', 'w') 

    #Top of tabular for LaTex file. 
    top_row = '\\hline\nEvaluation & Test Set & Re-training Type & Score 8-fold avg \\\\\n \\hline\n'

    wer_file.write(top_row)
    top_1_file.write(top_row)
    top_5_file.write(top_row)
    sem_partial_file_f1.write(top_row)
    sem_partial_file_p.write(top_row)
    sem_partial_file_r.write(top_row)
    sem_full_file.write(top_row)

    #Cleaner names for experimenents for tables. 
    exp_names = {'0.nbest': 'ASR & 0 & N/A',
                 '0.cky': 'CKY & 0 & N/A',
                 '1.nbest': 'ASR & 1 & N/A',
                 '1.cky': 'CKY & 1 & N/A',
                 '1.nbest_re_trained_baseline': 'ASR & 1 & ASR',
                 '1.nbest_re_trained_cky': 'ASR & 1 & CKY',
                 '1.cky_re_trained_baseline': 'CKY & 1 & ASR',
                 '1.cky_re_trained_cky': 'CKY & 1 & CKY'}


    for exp in experiment_names:
        wer_file.write(exp_names[exp] + ' & ' + str(results[exp]['wer']) + ' \\\\\n \\hline\n')
        top_1_file.write(exp_names[exp] + ' & ' + str(results[exp]['top_1']) + ' \\\\\n \\hline\n')
        top_5_file.write(exp_names[exp] + ' & ' + str(results[exp]['top_5']) + ' \\\\\n \\hline\n')
        sem_full_file.write(exp_names[exp] + ' & ' + str(results[exp]['sem_full']) + ' \\\\\n \\hline\n')

        #Writes sem partial results. 
        sem_partial_file_p.write(exp_names[exp] + ' & ' + str(results[exp]['sem_partial']['p']) + ' \\\\\n \\hline\n')
        sem_partial_file_r.write(exp_names[exp] + ' & ' + str(results[exp]['sem_partial']['r']) + ' \\\\\n \\hline\n')
        sem_partial_file_f1.write(exp_names[exp] + ' & ' + str(results[exp]['sem_partial']['f1']) + ' \\\\\n \\hline\n')

    wer_file.close()
    top_1_file.close()
    top_5_file.close()
    sem_partial_file_f1.close()
    sem_partial_file_p.close()
    sem_partial_file_r.close()
    sem_full_file.close()

"""
Extracts results from a tuning
experiment metric. 
"""
def extract_tuning_results(tuning_file_name): 
    tuning_file = open(tuning_file_name, 'r')

    weights = []
    avgs = []

    for line in tuning_file:
        weight, scores = line.strip().split(':')

        #Gets weight and scores for all folds. 
        weight = float(weight)
        scores = [float(element) for element in scores.split(';')]

        #Only interested in average performance. 
        avg_score = float(sum(scores)) / float(len(scores))
    
        #Package pair. 
        weights.append(weight)
        avgs.append(avg_score)

    #Return weights and associated avg score. 
    return [weights, avgs]

"""
Makes graphs for tuning results. 
"""
def plot_tuning_results(results_dir): 
    wer = extract_tuning_results(results_dir + 'wer.txt')
    top_1 = extract_tuning_results(results_dir + 'top_1.txt')
    top_5 = extract_tuning_results(results_dir + 'top_5.txt')
    sem_full = extract_tuning_results(results_dir + 'sem_full.txt')
    f1 = extract_tuning_results(results_dir + 'f1.txt')
    p = extract_tuning_results(results_dir + 'p.txt')
    r = extract_tuning_results(results_dir + 'r.txt')

    metric_names = {"wer": "WER", "r": "Recall", "p": "Precision", "f1": "F1", "sem_full": "Full Semantic Form",
                    "top_1": "Top 1", "top_5": "Top 5"}

    metric_results = {"wer": wer, "r": r, "p": p, "f1": f1, "sem_full": sem_full,
                    "top_1": top_1, "top_5": top_5}

    for metric in ['wer', 'sem_full', 'top_1', 'top_5', 'f1', 'p', 'r']:
        plt.plot(metric_results[metric][0], metric_results[metric][1], 'r')
        plt.xlabel(r'$\beta$', fontsize =24)
        plt.ylabel('Average Score', fontsize=24)
        plt.title(metric_names[metric], fontsize=26)
        plt.xticks(np.arange(0.0, 1.1, 0.1))
        plt.savefig(results_dir + metric + '.png')
        plt.close()


"""
Runs tuning experiments
for re-ranking with interpolation. 
"""
def run_tuning_experiments(experiment_dir_path):
    for fold_name in os.listdir(experiment_dir_path):
        #Fork to speed up processing. 
        pid = os.fork()

        if pid == 0: 
            result_path = experiment_dir_path + '/' + fold_name + '/tuning/results/'
            nbest_path = experiment_dir_path + '/' + fold_name + '/experiments/asr/result_files/0.nbest'

            #Weights to interpolate over. 
            weights = np.arange(0.0, 1.005, 0.005)

            for weight in weights:
                args = ['./experiments.py', 'rerank_interpolation']
                args.append(nbest_path)
                args.append(result_path + str(weight))
                args.append(str(weight))

                print "Fold " + str(fold_name) + " weight " + str(weight)
                subprocess.call(args)

            sys.exit()

        #Fork to speed up processing. 
        pid = os.fork()

        if pid == 0: 
            result_path = experiment_dir_path + '/' + fold_name + '/tuning1/results/'
            nbest_path = experiment_dir_path + '/' + fold_name + '/experiments/asr/result_files/1.nbest'

            #Weights to interpolate over. 
            weights = np.arange(0.0, 1.005, 0.005)

            for weight in weights:
                args = ['./experiments.py', 'rerank_interpolation']
                args.append(nbest_path)
                args.append(result_path + str(weight))
                args.append(str(weight))

                print "Fold " + str(fold_name) + " weight " + str(weight)
                subprocess.call(args)

            sys.exit()
"""
Evaluates tuning experiments. 
"""
def evaluate_tuning(experiment_dir_path):
    for fold_name in os.listdir(experiment_dir_path):
        pid = os.fork()

        if pid == 0:
            result_path = experiment_dir_path + '/' + fold_name + '/tuning/results/'
            evaluation_path = experiment_dir_path + '/' + fold_name + '/tuning/evaluations/'
            parser_path = experiment_dir_path + '/' + fold_name + '/models/parser.cky'

            for result_file in os.listdir(result_path):
                evaluate.wer(result_path + result_file, evaluation_path + '/wer/' + result_file, fold_name + '_' + result_file + '0')
                evaluate.correct_in_top_n(result_path + result_file, evaluation_path + '/top_1/' + result_file, 1)
                evaluate.correct_in_top_n(result_path + result_file, evaluation_path + '/top_5/' + result_file, 5)
                evaluate.semantic_form(result_path + result_file, evaluation_path + '/sem_full/' + result_file, parser_path)
                evaluate.semantic_form_partial(result_path + result_file, evaluation_path + '/sem_partial/' + result_file, parser_path)

            sys.exit()

        pid = os.fork()

        if pid == 0:
            result_path = experiment_dir_path + '/' + fold_name + '/tuning1/results/'
            evaluation_path = experiment_dir_path + '/' + fold_name + '/tuning1/evaluations/'
            parser_path = experiment_dir_path + '/' + fold_name + '/models/parser.cky'

            for result_file in os.listdir(result_path):
                evaluate.wer(result_path + result_file, evaluation_path + '/wer/' + result_file, fold_name + '_' + result_file + '1')
                evaluate.correct_in_top_n(result_path + result_file, evaluation_path + '/top_1/' + result_file, 1)
                evaluate.correct_in_top_n(result_path + result_file, evaluation_path + '/top_5/' + result_file, 5)
                evaluate.semantic_form(result_path + result_file, evaluation_path + '/sem_full/' + result_file, parser_path)
                evaluate.semantic_form_partial(result_path + result_file, evaluation_path + '/sem_partial/' + result_file, parser_path)

            sys.exit()
"""
Consolidates results form tuning experiments. 
"""
def consolidates_tuning(experiment_dir_path, consolidation_path):

    consolidate_path = consolidation_path + '/tuning/'
    scores = {}
    weights = np.arange(0.0, 1.005, 0.005)

    for fold_name in os.listdir(experiment_dir_path): 
        evaluation_path = experiment_dir_path + '/' + fold_name + '/tuning/evaluations/'

        for weight in weights:
            if not weight in scores: 
                scores[weight] = {'wer': [], 'sem_full': [], 'f1': [], 'p': [], 'r': [], 'top_1': [], 'top_5': []}

            scores[weight]['wer'].append(analysis.extract_wer_from_file(evaluation_path + 'wer/' + str(weight)))
            scores[weight]['top_1'].append(analysis.extract_topn_result_from_file(evaluation_path + 'top_1/' + str(weight)))
            scores[weight]['top_5'].append(analysis.extract_topn_result_from_file(evaluation_path + 'top_5/' + str(weight)))
            scores[weight]['sem_full'].append(analysis.extract_sem_full_avg_from_file(evaluation_path + 'sem_full/' + str(weight)))

            f1, p, r = analysis.extract_sem_partial_scores_from_file(evaluation_path + 'sem_partial/' + str(weight))
            scores[weight]['f1'].append(f1)
            scores[weight]['p'].append(p)
            scores[weight]['r'].append(r)

    #Now consolidates results into files.  
    wer_file = open(consolidate_path + 'wer.txt', 'w')
    top_1_file = open(consolidate_path + 'top_1.txt', 'w')
    top_5_file = open(consolidate_path + 'top_5.txt', 'w')
    sem_full_file = open(consolidate_path + 'sem_full.txt', 'w')
    f1_file = open(consolidate_path + 'f1.txt', 'w')
    p_file = open(consolidate_path + 'p.txt', 'w')
    r_file = open(consolidate_path + 'r.txt', 'w')

    for weight in weights:
        wer = [str(element) for element in scores[weight]['wer']]
        top_1 = [str(element) for element in scores[weight]['top_1']]
        top_5 = [str(element) for element in scores[weight]['top_5']]
        f1 = [str(element) for element in scores[weight]['f1']]
        p = [str(element) for element in scores[weight]['p']]
        r = [str(element) for element in scores[weight]['r']]
        sem_full = [str(element) for element in scores[weight]['sem_full']]

        wer_file.write(str(weight) + ':' + ';'.join(wer) + '\n')
        top_1_file.write(str(weight) + ':' + ';'.join(top_1) + '\n')
        top_5_file.write(str(weight) + ':' + ';'.join(top_5) + '\n')
        sem_full_file.write(str(weight) + ':' + ';'.join(sem_full) + '\n')
        f1_file.write(str(weight) + ':' + ';'.join(f1) + '\n')
        p_file.write(str(weight) + ':' + ';'.join(p) + '\n')
        r_file.write(str(weight) + ':' + ';'.join(r) + '\n')

    wer_file.close()
    top_1_file.close()
    top_5_file.close()
    sem_full_file.close()
    f1_file.close()
    p_file.close()
    r_file.close()

    consolidate_path = consolidation_path + '/tuning1/'
    scores = {}
    weights = np.arange(0.0, 1.005, 0.005)

    for fold_name in os.listdir(experiment_dir_path): 
        evaluation_path = experiment_dir_path + '/' + fold_name + '/tuning1/evaluations/'

        for weight in weights:
            if not weight in scores: 
                scores[weight] = {'wer': [], 'sem_full': [], 'f1': [], 'p': [], 'r': [], 'top_1': [], 'top_5': []}

            scores[weight]['wer'].append(analysis.extract_wer_from_file(evaluation_path + 'wer/' + str(weight)))
            scores[weight]['top_1'].append(analysis.extract_topn_result_from_file(evaluation_path + 'top_1/' + str(weight)))
            scores[weight]['top_5'].append(analysis.extract_topn_result_from_file(evaluation_path + 'top_5/' + str(weight)))
            scores[weight]['sem_full'].append(analysis.extract_sem_full_avg_from_file(evaluation_path + 'sem_full/' + str(weight)))

            f1, p, r = analysis.extract_sem_partial_scores_from_file(evaluation_path + 'sem_partial/' + str(weight))
            scores[weight]['f1'].append(f1)
            scores[weight]['p'].append(p)
            scores[weight]['r'].append(r)

    #Now consolidates results into files.  
    wer_file = open(consolidate_path + 'wer.txt', 'w')
    top_1_file = open(consolidate_path + 'top_1.txt', 'w')
    top_5_file = open(consolidate_path + 'top_5.txt', 'w')
    sem_full_file = open(consolidate_path + 'sem_full.txt', 'w')
    f1_file = open(consolidate_path + 'f1.txt', 'w')
    p_file = open(consolidate_path + 'p.txt', 'w')
    r_file = open(consolidate_path + 'r.txt', 'w')

    for weight in weights:
        wer = [str(element) for element in scores[weight]['wer']]
        top_1 = [str(element) for element in scores[weight]['top_1']]
        top_5 = [str(element) for element in scores[weight]['top_5']]
        f1 = [str(element) for element in scores[weight]['f1']]
        p = [str(element) for element in scores[weight]['p']]
        r = [str(element) for element in scores[weight]['r']]
        sem_full = [str(element) for element in scores[weight]['sem_full']]

        wer_file.write(str(weight) + ':' + ';'.join(wer) + '\n')
        top_1_file.write(str(weight) + ':' + ';'.join(top_1) + '\n')
        top_5_file.write(str(weight) + ':' + ';'.join(top_5) + '\n')
        sem_full_file.write(str(weight) + ':' + ';'.join(sem_full) + '\n')
        f1_file.write(str(weight) + ':' + ';'.join(f1) + '\n')
        p_file.write(str(weight) + ':' + ';'.join(p) + '\n')
        r_file.write(str(weight) + ':' + ';'.join(r) + '\n')

    wer_file.close()
    top_1_file.close()
    top_5_file.close()
    sem_full_file.close()
    f1_file.close()
    p_file.close()
    r_file.close()

"""
Prints the usage instructions  
for the script. 
"""
def print_usage():
    print 'Create n-fold experiment folder with all files needed for training: ./run_pipeline.py set_up [preprocessed_corpus_path] [n_fold_dir_path] [n]'
    print 'Train models: ./run_pipeline.py train [experiment_dir]'
    print 'Run 1st experiments (re-ranking on initial speech output): ./run_pipeline.py exp1 [experiment_dir]'
    print 'Run 2nd experiment (re-training w/ w/out re-ranking): ./run_pipeline.py exp2 [experiment_dir]'
    print 'Consolidate experiment results to LaTex: ./run_pipeline.py consolidate_latex [experiment_dir] [consolidated_files_dir]'
    print 'Consolidate experiment results to pyplot: ./run_pipeline.py consolidate_pyplot [experiment_dir] [consolidated_files_dir]'
    print 'Run tuning experiments: ./run_pipeline.py tuning_exp [experiment_dir]'
    print 'Evaluate tuning experiments: ./run_pipeline.py tuning_eval [experiment_dir]'
    print 'Consolidate tuning results: ./run_pipeline.py tuning_consolidate [experiment_dir] [consolidation_dir]'
    print 'Plot tuning results: ./run_pipeline.py tuning_plot [tuning_results_dir]'

if __name__ == '__main__':
    if len(sys.argv) >= 2:
        if sys.argv[1] == 'set_up':
            if len(sys.argv) == 5:
                set_up(sys.argv[2], sys.argv[3], sys.argv[4])
            else:
                print_usage()
        
        elif sys.argv[1] == 'train':
            if len(sys.argv) == 3:
                train_models(sys.argv[2])
            else:
                print_usage()

        elif sys.argv[1] == 'exp1':
            if len(sys.argv) == 3:
                exp1(sys.argv[2])
            else:
                print_usage()

        elif sys.argv[1] == 'exp2':
            if len(sys.argv) == 3:
                exp2(sys.argv[2])
            else:
                print_usage()

        elif sys.argv[1] == 'consolidate_latex':
            if len(sys.argv) == 4:
                consolidate_results_to_latex(sys.argv[2], sys.argv[3])
            else:
                print_usage()

        elif sys.argv[1] == 'consolidate_pyplot':
            if len(sys.argv) == 4:
                consolidate_results_to_pyplot(sys.argv[2], sys.argv[3])
            else:
                print_usage()

        elif sys.argv[1] == 'tuning_exp':
            if len(sys.argv) == 3:
                run_tuning_experiments(sys.argv[2])
            else:
                print_usage()

        elif sys.argv[1] == 'tuning_eval':
            if len(sys.argv) == 3:
                evaluate_tuning(sys.argv[2])
            else:
                print_usage()

        elif sys.argv[1] == 'tuning_consolidate':
            if len(sys.argv) == 4:
                consolidates_tuning(sys.argv[2], sys.argv[3])
            else:
                print_usage()

        elif sys.argv[1] == 'tuning_plot':
            if len(sys.argv) == 3:
                plot_tuning_results(sys.argv[2])
            else:
                print_usage()

        else: 
            print_usage()
    else:
        print_usage()

