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
from experiments import get_first_valid_parse
from experiments import tokenize_for_parser
import post_process
import pickle
import time

#Adds nlu_pipeline src folder in order to import modules from it. 
nlu_pipeline_path = '/scratch/cluster/rcorona/nlu_pipeline/src/'
sys.path.append(nlu_pipeline_path)

#Nlu pipeline modules.
try:
    import CKYParser
    from Grounder import Grounder
    from Lexicon import Lexicon
    from Ontology import Ontology
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
def wer(result_file_name, evaluation_file_name, temp_name=''):
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
    transcript_file = open(temp_name + 'transcripts.txt', 'w')
    hyp_file = open(temp_name + 'hyp.txt', 'w')

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
    args = [bin_path + '/word_align.pl', temp_name + 'transcripts.txt', temp_name + 'hyp.txt']

    #Evaluates WER. 
    subprocess.call(args, stdout=evaluation_file, stderr=evaluation_file)

    #Deletes temp files and closes evaluation. 
    os.remove(temp_name + 'transcripts.txt')
    os.remove(temp_name + 'hyp.txt')
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
            phrase, semantic_form, _ = line.strip().split('#')[1].split(';')

            #Line immediately after phrase is top scoring hypothesis. 
            line = result_file.readline().strip()
            hypothesis = line.split(';')

            data.append([semantic_form, hypothesis, phrase])

    #Loads parser. 
    parser = load_obj_general(parser_file)

    #Counts for evaluation. 
    count_correct = 0
    total_count = 0

    #Evaluates data set. 
    for truth_hyp_phrase in data:
        total_count += 1

        true_semantic_form, hyp, phrase = truth_hyp_phrase
        true_semantic_form = ':'.join(true_semantic_form.split(':')[1:])

        #Gets semantic node from true semantic form. 
        true_node = parser.lexicon.read_semantic_form_from_str(true_semantic_form, None, None, [], False)

        #Parses top hypothesis to get semantic node.
        if len(hyp) == 2: #In this case, only score and  phrase hyp are present. 
            parse = get_first_valid_parse(tokenize_for_parser(hyp[0]), parser)[0]

            #Continues of no valid parse was found for hypothesis. 
            if parse == None:
                continue
            else:
                hyp_node = parse.node

        #In this case, score, phrase, and semantic form are present, so check form is valid. 
        elif hyp[1] == "None":
            continue
        else:
            hyp_node = parser.lexicon.read_semantic_form_from_str(hyp[1], None, None, [], False)

        #Evaluates equality.
        are_equal = true_node.equal_allowing_commutativity(hyp_node, parser.commutative_idxs, True, parser.ontology)

        if are_equal:
            count_correct += 1


        #Prints information for the logs. 
        print '**********************************************'
        print "Ground truth phrase: " + phrase
        print "Ground truth node: " + str(true_node) 
        print "Ground truth semantic form: " + parser.print_parse(true_node)
        print ''

        print "Hyp str: " + hyp[0] 
        print "Hyp node: " + str(hyp_node)
        print "Hyp semantic form: " + parser.print_parse(hyp_node)
        print  ''

        print "Equal: " + str(are_equal)
        print '***********************************************'
        print ''

    #Writes evaluation results.
    evaluation_file = open(evaluation_file_name, 'w')
    evaluation_file.write("TOTAL:" + str(total_count) + '\n')
    evaluation_file.write("CORRECT:" + str(count_correct) + '\n')
    evaluation_file.close()

def semantic_form_partial(result_file_name, evaluation_file_name, parser_file): 
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
            phrase, semantic_form, _ = line.strip().split('#')[1].split(';')

            #Line immediately after phrase is top scoring hypothesis. 
            line = result_file.readline().strip()
            hypothesis = line.split(';')

            data.append([semantic_form, hypothesis, phrase])

    #Loads parser. 
    parser = load_obj_general(parser_file)

    #Counts for evaluation. 
    recall = 0.0
    precision = 0.0
    f1_score = 0.0
    total_count = 0.0

    #Evaluates data set. 
    for truth_hyp_phrase in data:
        total_count += 1.0

        true_semantic_form, hyp, phrase = truth_hyp_phrase
        true_semantic_form = ':'.join(true_semantic_form.split(':')[1:])

        #Gets semantic node from true semantic form. 
        true_node = parser.lexicon.read_semantic_form_from_str(true_semantic_form, None, None, [], False)

        #Parses top hypothesis to get semantic node.
        if len(hyp) == 2: #In this case, only score and  phrase hyp are present. 
            parse = get_first_valid_parse(tokenize_for_parser(hyp[0]), parser)[0]

            #Continues if no valid parse was found for hypothesis. 
            if parse == None:
                continue
            else:
                hyp_node = parse.node

        #In this case, score, phrase, and semantic form are present, so check form is valid. 
        elif hyp[1] == "None":
            continue
        else:
            hyp_node = parser.lexicon.read_semantic_form_from_str(hyp[1], None, None, [], False)

        #Evaluates recall and precision for hypothesis and adds values to counts. 
        true_preds = [triple[0] for triple in parser.theta.count_semantics(true_node)]
        hyp_preds = [triple[0] for triple in parser.theta.count_semantics(hyp_node)]
        true_args = [triple[1] for triple in parser.theta.count_semantics(true_node)]
        hyp_args = [triple[1] for triple in parser.theta.count_semantics(hyp_node)]

        hyp_precision = 0.0
        hyp_recall = 0.0

        #precision = # correct preds out of total guessed. 
        for hyp_pred in hyp_preds:
            if hyp_pred in true_preds:
                hyp_precision += 1.0

        #recall = # correct preds out of total number correct. 
        for true_pred in true_preds:
            if true_pred in hyp_preds:
                hyp_recall += 1.0

        #Adds values. 
        hyp_precision /= len(hyp_preds)
        hyp_recall /= len(true_preds)

        precision += hyp_precision
        recall += hyp_recall

        if not (hyp_precision == 0 and hyp_recall == 0):
            f1_score += 2 * hyp_precision * hyp_recall / (hyp_precision + hyp_recall)   
        

    #Normalizes
    precision /= total_count
    recall /= total_count
    f1_score /= total_count

    #Writes evaluation results.
    evaluation_file = open(evaluation_file_name, 'w')
    evaluation_file.write("AVG PRECISION:" + str(precision) + '\n')
    evaluation_file.write("AVG RECALL:" + str(recall) + '\n')
    evaluation_file.write("AVG F1:" + str(f1_score))
    evaluation_file.close()

def evaluate_parse_file_full(file_name, parser): 
    result_file = open(file_name, 'r')

    correct_count = 0
    total_count = 0

    for line in result_file:    
        true, hyp = line.strip().split(';')
        
        #Add type constraints if needed to the true semantic form. 
        true = post_process.add_type_constraints(true)

        if not hyp == 'None': 
            #Get semantic form nodes for hypothesis and ground  truth. 
            true_node = parser.lexicon.read_semantic_form_from_str(true, None, None, [], False)
            hyp_node = parser.lexicon.read_semantic_form_from_str(hyp, None, None, [], False)

            #Now check for equivalence. 
            are_equal = true_node.equal_allowing_commutativity(hyp_node, parser.commutative_idxs, True, parser.ontology)

            if are_equal: 
                correct_count += 1

        total_count += 1

    return float(correct_count) / float(total_count) 

def evaluate_parse_folder_full(experiment_folder, result_folder): 
    scores = []

    for file_name in os.listdir(result_folder): 
        #Get parser for file. 
        fold_name = file_name.split('.')[0]
        parser_path = experiment_folder + fold_name + '/models/parser.cky'
        parser = load_obj_general(parser_path)

        scores.append(evaluate_parse_file_full(result_folder + file_name, parser))

    print (float(sum(scores)) / float(len(scores)))

def evaluate_parse_file_partial(file_name, parser): 
    result_file = open(file_name, 'r')

    total_count = 0

    #Will hold return values. 
    precision = 0.0
    recall = 0.0
    f1_score = 0.0

    for line in result_file:    
        true, hyp = line.strip().split(';')
        
        #Add type constraints if needed to the true semantic form. 
        true = post_process.add_type_constraints(true) 

        if not hyp == 'None': 
            #Get semantic form nodes for hypothesis and ground  truth. 
            true_node = parser.lexicon.read_semantic_form_from_str(true, None, None, [], False)
            hyp_node = parser.lexicon.read_semantic_form_from_str(hyp, None, None, [], False)

            #Evaluates recall and precision for hypothesis and adds values to counts. 
            true_preds = [triple[0] for triple in parser.theta.count_semantics(true_node)]
            hyp_preds = [triple[0] for triple in parser.theta.count_semantics(hyp_node)]
            true_args = [triple[1] for triple in parser.theta.count_semantics(true_node)]
            hyp_args = [triple[1] for triple in parser.theta.count_semantics(hyp_node)]

            hyp_precision = 0.0
            hyp_recall = 0.0

            #precision = # correct preds out of total guessed. 
            for hyp_pred in hyp_preds:
                if hyp_pred in true_preds:
                    hyp_precision += 1.0

            #recall = # correct preds out of total number correct. 
            for true_pred in true_preds:
                if true_pred in hyp_preds:
                    hyp_recall += 1.0

            #Adds values. 
            hyp_precision /= len(hyp_preds)
            hyp_recall /= len(true_preds)

            precision += hyp_precision
            recall += hyp_recall

            if not (hyp_precision == 0 and hyp_recall == 0):
                f1_score += 2 * hyp_precision * hyp_recall / (hyp_precision + hyp_recall)

        total_count += 1

    #Normalizes
    precision /= total_count
    recall /= total_count
    f1_score /= total_count

    return [f1_score, precision, recall]

def eval_google_wer(results, parser): 
    #Creates temporary files to run evaluations on.
    os.chdir('../bin')
    transcript_file = open('transcripts.txt', 'w')
    hyp_file = open('hyp.txt', 'w')

    #Used for naming phrases, needed by wer script. 
    count = 0

    for target, hypothesis in results:
        transcript_file.write(target[0] + ' (phrase' + str(count) + ')\n')
        hyp_file.write(hypothesis[0] + ' (phrase' + str(count) + ')\n')

        count += 1

    #Closes files so they may be read by WER evaluation script. 
    transcript_file.close()
    hyp_file.close()

    #Prepares arguments for running WER evaluation. 
    args = [bin_path + '/word_align.pl', 'transcripts.txt', 'hyp.txt']

    #Evaluates WER. 
    subprocess.call(args, stdout=None, stderr=None)

def eval_google_partial_sem_form(results, parser): 
    #Counts for evaluation. 
    recall = 0.0
    precision = 0.0
    f1_score = 0.0
    total_count = 0.0

    #Evaluates data set. 
    for target, hypothesis in results:
        total_count += 1.0

        #Get the target semantic form. 
        _, true_semantic_form, _, _ = target
        true_semantic_form = ':'.join(true_semantic_form.split(':')[1:])
        true_semantic_form = post_process.add_type_constraints(true_semantic_form.strip()) 

        #Gets semantic node from true semantic form. 
        true_node = parser.lexicon.read_semantic_form_from_str(true_semantic_form, None, None, [], False)

        #Now attempt to get a semantic node from the semantic form string. 
        hyp_sem_form_str = hypothesis[2]

        #If None, then no valid parse was found. Therefore skip since this won't contribute to the score.  
        if hyp_sem_form_str == "None":
            continue
        else:
            hyp_node = parser.lexicon.read_semantic_form_from_str(hyp_sem_form_str, None, None, [], False)

        #Evaluates recall and precision for hypothesis and adds values to counts. 
        true_preds = [triple[0] for triple in parser.theta.count_semantics(true_node)]
        hyp_preds = [triple[0] for triple in parser.theta.count_semantics(hyp_node)]
        true_args = [triple[1] for triple in parser.theta.count_semantics(true_node)]
        hyp_args = [triple[1] for triple in parser.theta.count_semantics(hyp_node)]

        hyp_precision = 0.0
        hyp_recall = 0.0

        #precision = # correct preds out of total guessed. 
        for hyp_pred in hyp_preds:
            if hyp_pred in true_preds:
                hyp_precision += 1.0

        #recall = # correct preds out of total number correct. 
        for true_pred in true_preds:
            if true_pred in hyp_preds:
                hyp_recall += 1.0

        #Adds values. 
        hyp_precision /= len(hyp_preds)
        hyp_recall /= len(true_preds)

        precision += hyp_precision
        recall += hyp_recall

        if not (hyp_precision == 0 and hyp_recall == 0):
            f1_score += 2 * hyp_precision * hyp_recall / (hyp_precision + hyp_recall)   
        
    #Normalizes
    precision /= total_count
    recall /= total_count
    f1_score /= total_count

    #Now display results.
    """
    print 'P: ' + str(precision)
    print 'R: ' + str(recall)
    print 'F1: ' + str(f1_score)
    """

    #Return F1 score for evaluation. 
    return f1_score

def eval_google_full_sem_form(results, parser): 
    #Counts for evaluation. 
    count_correct = 0.0
    total_count = 0.0

    #Evaluates data set. 
    for target, hypothesis in results:
        total_count += 1.0
 
        _, true_semantic_form, _, _ = target
        true_semantic_form = ':'.join(true_semantic_form.split(':')[1:])
        true_semantic_form = post_process.add_type_constraints(true_semantic_form.strip()) 

        #Gets semantic node from true semantic form. 
        true_node = parser.lexicon.read_semantic_form_from_str(true_semantic_form, None, None, [], False)

        #Now attempt to get a semantic node from the semantic form string. 
        hyp_sem_form_str = hypothesis[2]

        #If None, then no valid parse was found. Therefore skip since this won't contribute to the score.  
        if hyp_sem_form_str == "None":
            continue
        else:
            hyp_node = parser.lexicon.read_semantic_form_from_str(hyp_sem_form_str, None, None, [], False)

        #Evaluates equality.
        are_equal = true_node.equal_allowing_commutativity(hyp_node, parser.commutative_idxs, True, parser.ontology)

        if are_equal:
            count_correct += 1.0

    accuracy = count_correct / total_count

    #Now display results. 
    #print 'Accuracy: ' + str(accuracy)

    return accuracy

def prepare_google_speech_results(raw_results, speech_scoring_function): 
    results = []

    #Targets will be paired with hypotheses. 
    target = None
    hypotheses = []

    for row in raw_results: 
        if row[0].startswith('#'): 
            #First append previous pair to results if it exists. 
            if target:
                #First re-score speech confidence scores using given scoring function. 
                hypotheses = speech_scoring_function(hypotheses)

                results.append((target, hypotheses))

            #New target to be read. 
            target = row

            #New hypothesis list to read in. 
            hypotheses = []

        #Otherwise, this is just another hypothesis to add. 
        else:
            hypotheses.append([row[0], float(row[1]), row[2], float(row[3])])

    return results

def re_rank_results(results, lambda1): 
    #Re-ranked results will only contain top ranked result. 
    re_ranked_results = []

    #Lambda 2 can be derived from lambda 1. 
    lambda2 = 1.0 - lambda1

    #Define a score function for each hypothesis. 
    #TODO add actual interpolation. Right now assumes we are either re-ranking based solely on parser or not at all. 
    if lambda1 == 1.0: 
        score = lambda x: x[1]
    elif lambda2 == 1.0: 
        score = lambda x: x[3]

    #Get top re-ranked hypothesis and append to list. 
    for target, hypotheses in results: 
        re_ranked_results.append((target, sorted(hypotheses, key=score, reverse=True)[0]))

    return re_ranked_results

"""
Evaluates the google speech results of a particular parser
parameterization over all folds of an experiment folder. 
"""
def evaluate_google_speech_results(experiment_folder, test_file_extension, parser_params, lambda1, eval_function='partial_semantic_form', speech_scoring_function=lambda x:x):

    #Keep results per fold. 
    results = []

    for fold in os.listdir(experiment_folder): 
        fold_path = experiment_folder + fold

        #Determine path for the result file and the parser using the parser parameterization. 
        result_file_path = fold_path + '/experiments/asr/result_files/' + parser_params + '.' + test_file_extension
        parser_path = fold_path + '/models/' + parser_params + '.cky'

        #Add result to list. 
        results.append(evaluate_google_speech_result(result_file_path, parser_path, lambda1, eval_function, speech_scoring_function))

    #Now average results and present. 
    average = float(sum(results)) / float(len(results))

    print 'Average performance using ' + eval_function + ' evaluation function: ' + str(average)
    print results

"""
Evaluates the results of a Google Speech
re-ranking experiment of an individual file 
given a metric and weighting between 
speech and semantic parsing confidence scores. 
"""
def evaluate_google_speech_result(results_file_path, parser_path, lambda1, eval_function='partial_sem_form', speech_scoring_function=lambda x:x): 
    #Read in result data. 
    raw_results = [line.strip().split(';') for line in open(results_file_path, 'r')]

    #TODO Actually add a speech scoring function. 
    if speech_scoring_function == 'None':
        speech_scoring_function = lambda x:x

    #Prepare data for evaluation.
    results = prepare_google_speech_results(raw_results, speech_scoring_function)

    #Load parser. 
    parser = load_obj_general(parser_path)

    #Re-rank results based on interpolation value. 
    re_ranked_results = re_rank_results(results, lambda1)

    #Determine evaluation function. 
    if eval_function == 'partial_sem_form':
        eval_function = eval_google_partial_sem_form
    elif eval_function == 'full_sem_form':
        eval_function = eval_google_full_sem_form
    elif eval_function == 'wer':
        eval_function = eval_google_wer
    else:
        raise NotImplementedError('Eval function ' + eval_function + ' currently has no implementation!')

    #Now run evaluation. 
    return eval_function(re_ranked_results, parser)

"""
Go through all folds in an experiment. Evaluate
parsing performance for a given parameterization 
over all folds and return the parameterization 
that results in the best performance along with
its scores. Does this for both partial and full
semantic form evaluations. 

result_file_extension denotes the file over which
we are comparing performance. 
"""
def find_best_parsing_performance(experiment_folder, result_file_extension):   
    #First get list of all parameterizations which have results over all folds. 
    folds = os.listdir(experiment_folder)

    #Holds all result files for each parameterization. 
    result_files = {}

    print 'Getting all result files for given extension...'

    for fold in folds: 
        #Path to where the result files should live. 
        results_folder = experiment_folder + fold + '/experiments/asr/result_files/'

        #Get number of test lines so that we can ensure each result file is actually finished. 
        test_file = experiment_folder + fold + '/experiments/asr/test_files/' + result_file_extension + '.txt' 
        num_test_lines = len([line for line in open(test_file, 'r')])

        #Get all files. 
        fold_result_files = os.listdir(results_folder)

        #Now prune the ones that don't have the extension we are looking for. 
        fold_result_files = [result_file for result_file in fold_result_files if result_file.split('.')[-1] == result_file_extension]

        #Make sure result file has same number of lines as the original test file (i.e. is not still being written to).
        fold_result_files = [result_file for result_file in fold_result_files if len([line for line in open(results_folder + result_file, 'r')]) == num_test_lines]

        #Now store the path to each result file with the same parameterization from the other folds. 
        for result_file in fold_result_files: 

            #Some result_files may contain periods before the file extension. 
            parameterization = '.'.join(result_file.split('.')[:-1])

            #Path to the file so we can open and evaluate it later. 
            file_path = results_folder + result_file

            if not parameterization in result_files: 
                result_files[parameterization] = []

            result_files[parameterization].append(file_path)

    #Now that we have all files sorted, prune the result_files that don't have results over all folds. 
    result_files = {key: result_files[key] for key in result_files if len(result_files[key]) == len(folds)}

    print 'Getting partial semantic form scores...'

    #Collect all average partial semantic form scores for each parameterization.
    partial_sem_scores = {key: {'f1': [], 'p': [], 'r': []} for key in result_files.keys()}

    for parameterization in partial_sem_scores:  

        #Load the parser for the parameterization. 
        parser_path = experiment_folder + fold + '/models/' + parameterization + '.cky'
        parser = load_obj_general(parser_path)

        #Get score for each result file pertaning to parameterization. 
        for result_file in result_files[parameterization]:      
            print result_file
            f1, p, r = evaluate_parse_file_partial(result_file, parser)

        partial_sem_scores[parameterization]['f1'].append(f1)
        partial_sem_scores[parameterization]['p'].append(p)
        partial_sem_scores[parameterization]['r'].append(r)

    print 'Averaging partial semantic form scores...'

    #Averaging function
    avg = lambda x: float(sum(x)) / float(len(x))

    partial_sem_scores = {key: {'f1': avg(partial_sem_scores[key]['f1']), 'p': avg(partial_sem_scores[key]['p']), 'r': avg(partial_sem_scores[key]['r'])} for key in result_files.keys()}

    #Get best performing parameterization on F1 measure.
    best_f1 = float('-inf')
    best_f1_params = None

    for params in partial_sem_scores: 
        if partial_sem_scores[params]['f1'] > best_f1:
            best_f1 = partial_sem_scores[params]['f1']
            best_f1_params = params

    #Now get full semantic form scores. 
    print 'Getting full semantic form scores...'

    full_sem_scores = {key: [] for key in result_files.keys()}

    for parameterization in full_sem_scores:  

        #Load the parser for the parameterization. 
        parser_path = experiment_folder + fold + '/models/' + parameterization + '.cky'
        parser = load_obj_general(parser_path)

        #Get score for each result file pertaning to parameterization. 
        for result_file in result_files[parameterization]:       
            print result_file
            full_sem_scores[parameterization].append(evaluate_parse_file_full(result_file, parser))

    print 'Averaging full semantic form scores...'

    #Averaging function
    avg = lambda x: float(sum(x)) / float(len(x))

    full_sem_scores = {key: avg(full_sem_scores[key]) for key in result_files.keys()}

    #Get best performing parameterization on F1 measure.
    best_full_score = float('-inf')
    best_full_params = None

    for params in full_sem_scores: 
        if full_sem_scores[params] > best_full_score:
            best_full_score = full_sem_scores[params]
            best_full_params = params

    print 'Best F1: ' + str(best_f1)
    print 'Best F1 parameters: ' + best_f1_params

    print 'Best full score: ' + str(best_full_score)
    print 'Best full parameters: ' + best_full_params


def grounded_forms(ont_file, lex_file, kb_pickle):
    #Loads information needed for grounding. 
    ontology = Ontology(ont_file)
    lexicon = Lexicon(ontology, lex_file) 
    kb_predicates = load_obj_general(kb_pickle)
    
    grounder = Grounder(ontology, perception_module=None, kb_predicates=kb_predicates, classifier_predicates=None, test_features_file=None)

    examples_reader = open('ground.txt')
    lines = examples_reader.readlines()
    examples_reader.close()
    
    examples = list()
    i = 0
    while i < len(lines) :
        if len(lines[i].strip()) == 0 :
            i += 1
            continue
            
        text = lines[i].strip()
        ccg_str, form_str = lines[i+1].strip().split(" : ")
        ccg = lexicon.read_category_from_str(ccg_str)
        form = lexicon.read_semantic_form_from_str(form_str, None, None, [], allow_expanding_ont=False)
        form.category = ccg
        grounding = lines[i+2].strip()
        examples.append((text, form, grounding))
        i += 3
    
    print kb_predicates

    for (text, sem_form, grounding) in examples :
        print text
        print 'True grounding = ', grounding
        groundings, lambda_assignments = grounder.ground_semantic_node(sem_form)
        grounding_strs = [(str(grounding), prob) for (grounding, prob) in groundings]
        print 'Predicted groundings = ', grounding_strs
        print 'Lambda assignemnts = ', lambda_assignments
        print 

def eval_asr_length(experiment_folder, result_file_name): 
    averages = {}

    for fold_name in os.listdir(experiment_folder):
        #Open test file and its pertaining WER file for comparison. 
        test_file_path = experiment_folder + fold_name + '/experiments/asr/test_files/' + result_file_name + '.test'
        wer_file_path = experiment_folder + fold_name + '/evaluations/wer/' + result_file_name + '.nbest'

        test_file = open(test_file_path, 'r')
        wer_file = open(wer_file_path, 'r')

        #Will contain scores per phrase length.
        scores = {}

        #Line buffers. 
        test_line = test_file.readline()
        wer_line = wer_file.readline()

        while not test_line == '':
            #Get test phrase length. 
            phrase = test_line.split(';')[0]
            phrase_len = len(tokenize_for_parser(phrase).split())

            #Find line where WER is held for phrase. 
            while not wer_line.startswith('Words'):
                wer_line = wer_file.readline()

            #Extract WER. 
            wer = float(wer_line.split('Error = ')[1].split('%')[0])

            #Add WER to list of scores for phrase length. 
            if not phrase_len in scores: 
                scores[phrase_len] = []

            scores[phrase_len].append(wer)

            #Move lines forward. 
            wer_line = wer_file.readline()
            test_line = test_file.readline()

        #Now average and add to experiment-wide values. 
        for length in scores: 
            average = float(sum(scores[length])) / float(len(scores[length]))

            if not length in averages:
                averages[length] = []

            averages[length].append(average)

    results = {}

    #Now average over all folds for each length. 
    for length in sorted(averages, key=lambda key: averages[key]): 
        average = float(sum(averages[length])) / float(len(averages[length]))

        results[length] = average

    #Now print performance for each length. 
    for length in results: 
        print str(length) + ':' + str(results[length])

def eval_parsing_time(parser_path, test_file_path, chkpt_file_path, log_file_path=None):
    #First open up files, particularly, read in checkpoint in case we've already worked on this file. 
    test_file = [line for line in open(test_file_path, 'r')]
    
    #Get checkpoint if it already exists. 
    if os.path.exists(chkpt_file_path): 
        chkpt = pickle.load(open(chkpt_file_path, 'r'))
    else:
        chkpt = {'lines_parsed': set()}

    #Log everything for analysis later. 
    if not log_file_path: 
        log_file_path = os.path.splitext(chkpt_file_path)[0] + '.log'

    if os.path.exists(log_file_path):
        log_file = open(log_file_path, 'a')
    else:
        log_file = open(log_file_path, 'w')

    #Finally, load the parser. 
    parser = load_obj_general(parser_path)

    not_done = True
    tok_len = 0

    output_str = 'Starting from checkpoint: ' + str(chkpt)
    print output_str
    log_file.write(output_str + '\n')
    log_file.flush()

    #Do this for phrases of all lengths starting from the shortest. 
    while not_done: 
     
        tok_len += 1

        #Keep track of which lines we've done. 
        index = 0

        for line in test_file:
            #Tokenize input to parser. 
            phrase = tokenize_for_parser(line.split(';')[0])

            #Get number of tokens to store statistic in checkpoint dictionary. 
            num_toks = len(phrase.split())

            #Only parse sentences with num_toks tokens so that we can go from shortest to longest. 
            if num_toks == tok_len and not index in chkpt['lines_parsed']: 
                #Now time to see how long it takes to parse. 
                start_time = time.time()
                parse = parser.most_likely_cky_parse(phrase).next()
                end_time = time.time()

                #Get string representation of parsed semantic form.
                if parse == None or parse[0] == None: 
                    parse = 'None'
                else: 
                    parse_node = parse[0].node
                    parse = parser.print_parse(parse_node)
                
                duration = end_time - start_time

                #Now store that data. 
                if not num_toks in chkpt:
                    chkpt[num_toks] = []

                chkpt[num_toks].append([phrase, parse, duration])
                chkpt['lines_parsed'].add(index)

                #Save checkpoint. 
                pickle.dump(chkpt, open(chkpt_file_path, 'w'))

                output_str = 'Parsed phrase #' + str(index) + ', with ' + str(num_toks) + ' tokens, in ' + str(duration) + ' seconds.'
                print output_str
                log_file.write(output_str + '\n')
                log_file.flush()

                #Check to see if number of parsed lines matches test file, if so, we're done. 
                if len(chkpt['lines_parsed']) == len(test_file):
                    not_done = False

            index += 1

    output_str = 'Done.'
    print output_str
    log_file.write(output_str)

    #Done loggging. 
    log_file.close()

def print_usage():
    print 'WER: ./evaluate.py wer [result_file] [evaluation_file_name]'
    print 'Correct hypothesis in top n: ./evaluate.py top_n [result_file] [evaluation_file_name] [n]'
    print 'Semantic form evaluation: ./evaluate.py semantic_form [result_file] [evaluation_file_name] [parser] [full/partial]'
    print 'Grounding: ./evaluate.py grounding [ont file] [lex file] [kb pickle file]'
    print 'Evaluate parsing: ./evaluate.py evaluate_parsing [experiment_folder] [result_file_extension]'
    print 'Evaluate ASR per phrase length: ./evaluate asr_len [experiment_folder] [result_file_name]'
    print 'Evaluate Google Speech Results: ./evaluate google_speech [experiment_folder] [test_file_extension] [parser_params] [lambda1] [eval_function] [speech_scoring_function]'
    print 'Evaluate parsing time: ./evaluate parsing_time [parser_path] [test_file] [checkpoint_file]'

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
        if not len(sys.argv) == 6:
            print_usage()
        elif sys.argv[5] == 'full':
            semantic_form(sys.argv[2], sys.argv[3], sys.argv[4])
        elif sys.argv[5] == 'partial':
            semantic_form_partial(sys.argv[2], sys.argv[3], sys.argv[4])
        else:
            print_usage()

    elif sys.argv[1] == 'grounding':
        if len(sys.argv) == 5:
            grounded_forms(sys.argv[2], sys.argv[3], sys.argv[4])
        else:
            print_usage()

    elif sys.argv[1] == 'evaluate_parsing': 
        if len(sys.argv) == 4:
            find_best_parsing_performance(sys.argv[2], sys.argv[3])
        else:
            print_usage()

    elif sys.argv[1] == 'asr_len':
        if len(sys.argv) == 4:
            eval_asr_length(sys.argv[2], sys.argv[3])
        else:
            print_usage()

    elif sys.argv[1] == 'google_speech': 
        if not len(sys.argv) == 8:
            print_usage()
        else:
            evaluate_google_speech_results(sys.argv[2], sys.argv[3], sys.argv[4], float(sys.argv[5]), sys.argv[6], sys.argv[7])

    elif sys.argv[1] == 'parsing_time':
        if not len(sys.argv) == 5:
            print_usage()
        else:
            eval_parsing_time(sys.argv[2], sys.argv[3], sys.argv[4])

    else:
        print_usage()
