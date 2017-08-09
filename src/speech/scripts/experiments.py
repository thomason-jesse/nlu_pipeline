#!/usr/bin/python

"""
This script contains functions used for 
conducting ASR experiments, such as getting
the n-best hypothesis from Spinx, re-ranking,
etc. 

Usage
--------
ASR Nbest: ./experiments asr_n_best [sphinx_shared_library] [ac_model] [lm] [dict] [test_file] [nbest_file] [n]

    Parameters:
        sphinx_shared_library       - Path to the shared library which contains all the functions from sphinx, which
                                      this script wraps. 

        ac_model                    - Path to the acoustic model you wish to use. 

        lm                          - Path to the language model you would like to use. 

        dict                        - Path to the dictionary file you would like to use. 

        test_file                   - Path to the test file on which to perform the nbest hypothesis generation. 

        nbest_file                  - The file in which to store the nbest results. 

        n                           - The number of hypotheses to generate per phrase in the test file. 

Author(s): Rodolfo Corona, rcorona@utexas.edu | Aishwarya Padmakumar, aish@cs.utexas.edu
"""

import ctypes
import os
import sys
import numpy as np
import math
import time
import multiprocessing
import subprocess
import evaluate

#TODO Change if necessary. 
#Adds nlu_pipeline src folder in order to import modules from it. 
nlu_pipeline_path = '/scratch/cluster/rcorona/nlu_pipeline/src/'
sys.path.append(nlu_pipeline_path)

#Nlu pipeline modules.
import CKYParser
from CKYParser import count_ccg_productions
from utils import *



"""
This class is used to perform experiments with the
Sphinx ASR system, such as getting an n-best hypothesis list. 
"""
class SphinxExperiment:
    """
    Loads the pocketsphinx shared library
    in order to be able to use the system. 
    """
    def __init__(self, sphinx_lib):
        #Loads handle to library. 
        self.lib_handle = ctypes.CDLL(sphinx_lib)

        #Sets argument types for pocketsphinx functions. 
        self.lib_handle.sphinx_init.argtypes = (ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p)
        self.lib_handle.sphinx_close.argtypes = ()
        self.lib_handle.sphinx_n_best.argtypes = (ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int)

    """
    Initializes the pocketsphinx decoder using 
    a given acoustic model, language model, and
    dictionary. 
    """
    def init_decoder(self, ac_model, lm, dictionary):
        self.lib_handle.sphinx_init(ctypes.c_char_p(ac_model), ctypes.c_char_p(lm), ctypes.c_char_p(dictionary))

    """
    Closes decoder and cleans up memory. 
    """
    def close_decoder(self):
        self.lib_handle.sphinx_close()

    """
    Reads through a test file to get phrases and their 
    recordings. For each one gets n-best hypotheses using
    Sphinx and writes them to file along with the ground
    truth phrase. 
    """
    def n_best(self, test_file_name, nbest_result_file_name, n):
        #Converts in case it is still a string. 
        n = int(n)
    
        #Opens test file for reading. 
        test_file = open(test_file_name, 'r')

        #Creates nbest file before running speech recognition. 
        open(nbest_result_file_name, 'w').close()

        #Gets nbest hypotheses for each phrase in test file. 
        for line in test_file:
            #Gets phrase and pertinent recording file name from line. 
            phrase, semantic_form, denotation, recording_file_name = line.strip().split(';')

            #Formats ground truth to feed to Sphinx. 
            ground_truth = ';'.join([phrase, semantic_form, denotation])

            #Runs Sphinx ASR on it to get n-best hypotheses. 
            self.lib_handle.sphinx_n_best(ctypes.c_char_p(ground_truth), ctypes.c_char_p(recording_file_name), ctypes.c_char_p(nbest_result_file_name), ctypes.c_int(n))

###############################################################################
######################### Parser re-ranking functions #########################
###############################################################################

"""
This function takes in a string
and returns a tokenized version for 
it that is compatible with the format
the CKYParser expects. 
"""
def tokenize_for_parser(phrase):
    #Currently only splits possessive and removes periods. 
    return phrase.replace("'", " '").replace('.', '').replace('&', 'and').lower()

"""
This function takes a tokenized string
that was used by the CKYParser and
returns an un-tokenized version of it. 
"""
def un_tokenize_from_parser(phrase):
    return phrase.replace(" '", "'")

"""
Gets score of hypothesis element.
Used for CKY re-ranking. 
"""
def get_hyp_score(hypothesis):
    return hypothesis[1][1]

"""
Gets first first element of
a list and returns it. 
"""
def get_second_element(l):
    return l[1]

"""
Returns the last element of a list. 
"""
def get_last_element_as_float(l):
    return float(l[-1])

def time_out_helper(time_limit): 
    time.sleep(time_limit)

def timed_get_first_valid_parse(phrase, parser, time_limit): 
    #This will keep the parse to be returned. 
    parse_storage_queue = multiprocessing.Queue()

    #Put all arguments in the storage queue. 
    parse_storage_queue.put([phrase, parser])

    #Attempt to get parse within time limit. 
    parsing_thread = multiprocessing.Process(target=get_first_valid_parse_helper, args=(parse_storage_queue,))
    parsing_thread.start()

    #Time out thread. 
    time_out_thread = multiprocessing.Process(target=time_out_helper, args=(time_limit,))
    time_out_thread.start()

    #Now wait for one of the two to finish and return result. 
    waiting = True

    while waiting: 
        if not parsing_thread.is_alive(): 
            time_out_thread.terminate()
            waiting = False
        
        elif not time_out_thread.is_alive(): 
            parsing_thread.terminate()
            waiting = False

    if parse_storage_queue.empty():
        return (None, float('-inf')) 
    else: 
        return parse_storage_queue.get()

    return parse

def get_first_valid_parse_helper(parse_storage_queue): 
    phrase, parser = parse_storage_queue.get()

    parse_storage_queue.put(get_first_valid_parse(phrase, parser, float('inf')))

"""
Returns the best valid parse found given a parser
and a phrase. Returns None if no valid parse is 
found within given constraints.

NOTE: Expects phrase to already be tokenized for parser. 
"""
def get_first_valid_parse(phrase, parser, num_max_phrases=20):
    if len(phrase) == 0 :
        return (None, float('-inf'))

    #TODO Consider adding check for number of unknown words. 

    try:    
        parse_generator = parser.most_likely_cky_parse(phrase)
        num_parses_examined = 0
            
        for (parse, score, _) in parse_generator:
            num_parses_examined += 1
                
            if num_parses_examined == num_max_phrases:
                break
           
            elif parse is None:
                break
                
            elif parse.node.is_lambda and parse.node.is_lambda_instantiation :
                # Lambda headed parses are very unlikely to be correct. Drop them
                continue
                
            top_level_category = parser.lexicon.categories[parse.node.category]
               
            # M - imperative (full action), C - confirmation, NP - noun phrase for params
            if not top_level_category == 'M':
                continue
          
            elif parser.print_parse(parse.node).startswith('and'):
                continue
        
            #Final check to make sure parse follows parser's lexicon rules. 
            try:
                #This will fail if parse is not valid. 
                parser.lexicon.read_semantic_form_from_str(parser.print_parse(parse.node), None, None, [], False)

                return (parse, score)
            except: 
                continue 
        
    except (KeyboardInterrupt, SystemExit):
        raise

    """
    Might need this to handle other errors???
    except:
        error = str(sys.exc_info()[0])
            
        if self.error_log is not None :
            self.error_log.write('Parser error for string: ' + response)
            self.error_log.write(error + '\n')
            self.error_log.write(traceback.format_exc() + '\n\n\n')
            self.error_log.flush() 
       
        print traceback.format_exc()
    """

    #No valid parse found within beam. 
    return (None, float('-inf'))

#Computes values needed for incorporating NULL nodes into parse scores. 
def compute_parser_null_node_values(parser):
    #p(null) i.e. the prior on a null token. 
    parser.theta.null_prior = float('-inf')

    parser.theta.CCG_given_token #p(c|s)
    parser.theta.lexicon.entries # |SF|
    parser.theta.lexicon.categories #|CCG| 

    #Computes sum _c sum _s p(c|s)
    for pair in parser.theta.CCG_given_token:
        parser.theta.null_prior = np.logaddexp(parser.theta.null_prior, parser.theta.CCG_given_token[pair])

    #Normalizes by |SF| * |CCG| #Avg. Production probability (atomic)
    parser.theta.null_prior -= np.log(len(parser.theta.lexicon.entries)) + np.log(len(parser.theta.lexicon.categories))

    #computes p(root|root, null) for each CCG category i.e. the contribution from adding the null node to the root.
    parser.theta.null_conditional_prob = {}

    for i in range(len(parser.theta.lexicon.categories)):
        #Starts with log probability of 0. 
        parser.theta.null_conditional_prob[i] = float('-inf')

        for j in range(len(parser.theta.lexicon.categories)):
            #Gets production probability p(root ccg | root ccg, c) for every CCG category c. 
            if (i, i, j) in parser.theta.CCG_production:
                log_prob = parser.theta.CCG_production[(i, i, j)]
            else:
                log_prob = float('-inf')

            #Adds probability to sum. 
            parser.theta.null_conditional_prob[i] = np.logaddexp(parser.theta.null_conditional_prob[i], log_prob) 

        #Normalizes (i.e. divides by |CCG| ). 
        parser.theta.null_conditional_prob[i] -= np.log(len(parser.theta.lexicon.categories))

    #Computes p(_ | _, null) i.e. the average probability of a production. #Avg. probability of a production (category)
    parser.theta.null_average_production_prob = float('-inf')

    for i in range(len(parser.theta.lexicon.categories)):
        for j in range(len(parser.theta.lexicon.categories)):
            for k in range(len(parser.theta.lexicon.categories)): 
                #Gets probability for category. 
                if (i, j, k) in parser.theta.CCG_production:
                    log_prob = parser.theta.CCG_production[(i, j, k)]
                else:
                    log_prob = float('-inf')

                parser.theta.null_average_production_prob = np.logaddexp(parser.theta.null_average_production_prob, log_prob)

    #Normalizes
    parser.theta.null_average_production_prob -= np.log(len(parser.theta.lexicon.categories) ** 3)

"""
Re-ranks a result file using a given function. 
"""
def re_rank_function(in_file_name, out_file_name, parser_path, function):
    if function == 'null_node':
        re_rank_null_node(in_file_name, out_file_name, parser_path)

"""
Gets the average probability of the
productions in a given parse. 
"""
def get_avg_parse_production_prob(parser, parse):
    #Gets semantic node using semantic form string. 
    parse_node = parser.lexicon.read_semantic_form_from_str(parse, None, None, [], False)
 
    print (count_ccg_productions(parser.most_likely_cky_parse("go to ray 's office").next()[0]))

    sys.exit()

"""
Re-ranks a CKY re-ranked file's results
with the incorporation of null nodes
in order to better compare phrases
of different token lengths. 
"""
def re_rank_null_node(in_file_name, out_file_name, parser_path, temp_name=''):
    in_file = open(in_file_name, 'r')

    #Loads parser
    parser = load_obj_general(parser_path)

    #Computes values needed for null node contribution. 
    compute_parser_null_node_values(parser)

    #Keeps track of data. 
    data = []
    hypotheses = None
    ground_truth = None

    #Limits the number of hypotheses to re-rank.
    limit = 10

    for line in in_file:
        #Delimits new phrase. 
        if line.startswith('#'):
            counter = 0

            #Adds last phrase and hypotheses if it exists. 
            if not (hypotheses == None or ground_truth == None):
                data.append([ground_truth, hypotheses])

            ground_truth = line.strip().split('#')[1]
            
            #Starts new list of hypotheses. 
            hypotheses = []
        else:
            counter += 1
    
            #Gets hypothesis and feeds it to parser to get top scoring parse score. 
            hypothesis_values = line.strip().split(';')
            hypothesis = hypothesis_values[0]

            tokenized_hypothesis = tokenize_for_parser(hypothesis)

            #Gets number of tokens in hypothesis. 
            num_toks = len(tokenized_hypothesis.split())

            #Hypotheses greater than 7 tokens in length are not considered. 
            if counter > limit or num_toks > 7:
                parse_score = float('-inf')
            else:
                #Determines number of null nodes to add to parse score. 
                num_nulls = 7 - num_toks

                #Computes new parse score with added null nodes. 
                parse_score = float(hypothesis_values[-1]) + num_nulls * (parser.theta.null_prior + get_avg_parse_production_prob(parser, hypothesis_values[1]))#parser.theta.null_average_production_prob)
            
            hypothesis_values[-1] = str(parse_score)

            hypotheses.append(hypothesis_values)

    #Gets the last hypothesis. 
    if not (hypotheses == None or ground_truth == None):
        data.append([ground_truth, hypotheses])

    #Reranks list.
    for truth_hyp_list in data:
        truth_hyp_list[1] = sorted(truth_hyp_list[1], key=get_last_element_as_float, reverse=True)
        
    #Writes re-ranked hypotheses to new file. 
    re_ranked_file = open(out_file_name, 'w')
    
    for truth_hyp_list in data:
        #Writes ground truth phrase. 
        re_ranked_file.write('#' + truth_hyp_list[0] + '\n')

        #Writes hypotheses. 
        for hypothesis in truth_hyp_list[1]:
            re_ranked_file.write(';'.join(hypothesis) + '\n')

    re_ranked_file.close()

"""
Internal method which re-ranks a hypothesis
list by interpolating between the ASR scores
and the parse scores. 
"""
def re_rank_hypotheses_with_interpolation(hyp_buff, weight):
    #Will keep hypotheses and their scores in order to sort them at then end. 
    hyps_and_scores = []

    #Get all ASR and parse scores. 
    asr_scores = [float(hyp.split(';')[3]) for hyp in hyp_buff]
    parse_scores = [float(hyp.split(';')[2]) for hyp in hyp_buff]

    #Now find sums so that we may normalize to form a probability distribution. 
    asr_sum = float('-inf')
    parse_sum = float('-inf')

    for score in asr_scores:
        asr_sum = np.logaddexp(asr_sum, score)

    for score in parse_scores:
        parse_sum = np.logaddexp(parse_sum, score)

    for hyp in hyp_buff: 
        #Get scores in order to interpolate. 
        _, _, parse_score, asr_score = hyp.split(';')

        parse_score = float(parse_score)
        asr_score = float(asr_score)

        #Subtract logarithm to get log-scale probability.
        parse_score = parse_score - parse_sum
        asr_score = asr_score - asr_sum
    
        #Some times all parses are -inf
        if math.isnan(parse_score) or parse_score == float('-inf') or weight == 0.0:
            interpolated_score = np.log(1.0 - weight) + asr_score
        elif weight == 1.0:
            interpolated_score = np.log(weight) + parse_score
        else:
            #Now interpolate and assign to hypothesis. 
            interpolated_score = np.logaddexp(np.log(weight) + parse_score, np.log(1.0 - weight) + asr_score)
       
        hyps_and_scores.append([hyp, interpolated_score])

    #Sorts list. 
    hyps_and_scores = sorted(hyps_and_scores, key=get_second_element, reverse=True)

    return [hyp[0] for hyp in hyps_and_scores]

"""
Take in two lists of logprob scores
and return a list of interpolated scores
in the same order. 
"""
def interpolate_score_lists(list1, list2, lambda1): 
    assert(len(list1) == len(list2))

    #Lambda 2 can be derived from first lambda. 
    lambda2 = 1.0 - lambda1

    #Turn weights into log form for logspace calculations. 
    lambda1 = np.log(lambda1) if lambda1 > 0.0 else float('-inf')
    lambda2 = np.log(lambda2) if lambda2 > 0.0 else float('-inf')

    #Sums start at 0 (-inf in logspace).
    sum1 = float('-inf')
    sum2 = float('-inf')

    for score in list1:
        sum1 = np.logaddexp(sum1, score) if not score == float('-inf') else sum1

    for score in list2:
        sum2 = np.logaddexp(sum2, score) if not score == float('-inf') else sum2

    interpolated_scores = []

    for i in range(len(list1)):

        #Subtract logarithm to get log-scale probability.
        score1 = list1[i] - sum1
        score2 = list2[i] - sum2

        list1[i] = score1
        list2[i] = score2

        #If we subtracted -inf from any score, then it'll be nan. 
        score1 = float('-inf') if math.isnan(score1) else score1
        score2 = float('-inf') if math.isnan(score2) else score2

        #If one of the lambdas is 0, then we don't have to interpolate. 
        if lambda1 == float('-inf'):
            interpolated_score = score2
        elif lambda2 == float('-inf'): 
            interpolated_score = score1
        else:

            #If one of the scores is 0, then the interpolated score must equal the other. 
            if score1 == float('-inf'):
                interpolated_score = score2 + lambda2
            elif score2 == float('-inf'):
                interpolated_score = score1 + lambda1
            else: 
                #Actually interpolate.  
                interpolated_score = np.logaddexp(score1 + lambda1, score2 + lambda2)
           
        interpolated_scores.append(interpolated_score)

    return [interpolated_scores, list1, list2]

def surrogate_score(score_list): 
    #First element should have a confidence score. 
    assert(score_list[0] > 0)

    #All other elements should have 0.0 confidence scores. 
    for element in score_list[1:-1]:
        assert(element == 0.0)

    #Give inverse squared ranks. 
    rank = 2

    #Get weights for distributing probability.
    weights = [1.0 / (rank ** i) for i in range(1, len(score_list))]
    weights = [element / sum(weights) for element in weights]
    weights = [np.log(element) for element in weights]

    #Get probability that is left. 
    prob = np.log(1.0 - score_list[0])

    #Now distribute probability. 
    surrogate_scores = [prob + weights[i] for i in range(len(weights))] 
    surrogate_scores += [np.log(score_list[0])]
    
    return surrogate_scores

"""
Process Google Speech scores to do two things. 
1. Put them in logspace. 
2. Assign surrogate scores in case only the top hypothesis has a confidence score. 
"""
def process_speech_scores(score_list): 
    #First determine if we need to do surrogate scoring. 
    if 0.0 in score_list:
        return surrogate_score(score_list)
    else:
        return [np.log(score) for score in score_list]


"""
Normalize parser scores based on phrase lenght. 
"""
def process_parse_scores(parse_scores, phrases, parser): 

    assert(len(parse_scores) == len(phrases))

    for i in range(len(parse_scores)):
        num_nulls = 16 - len(tokenize_for_parser(phrases[i]).split())
        num_nulls = 0 if num_nulls < 0 else num_nulls

        parse_scores[i] += num_nulls * (parser.theta.null_prior + parser.theta.null_average_production_prob)

    return parse_scores

def timeout_proportion(results, parser):
    #Keep total count vs timeout count. 
    total  = 0.0
    timeouts = 0.0

    #Also keep track of number of hypothesis lists we were unable to parse. 
    num_unparsed_lists = 0.0

    for result in results:
        incorrect = (evaluate.eval_google_full_sem_form([result], parser) == 0.0)

        if incorrect: 
            target, hypotheses = result

            #Evaluate based on top-ranked result. 
            sem_form = hypotheses[0][2]
            time = hypotheses[0][4]

            if sem_form == 'None' and time >= 10.0:
                timeouts += 1.0
            
            total += 1.0

    return [timeouts / total, timeouts  / float(len(results))]

def average_interpolated_scores(folds, parsers, parser_score_lines_files, lm_score_lines_files, alpha, beta):
               
    f1_scores = []
    acc_scores = []
    wer_scores = []
    timeouts_failures = []
    timeouts_total = []

    for fold in folds: 
        #Access the data we need to process everything. 
        parser = parsers[fold]
        lm_score_lines = lm_score_lines_files[fold]
        parser_score_lines = parser_score_lines_files[fold]

        #Prepare parser for normalization based on phrase length. 
        compute_parser_null_node_values(parser)

        assert(len(parser_score_lines) == len(lm_score_lines))

        results = []
        unranked_results = []
        target = None

        #Prepare results to re-rank with alpha.
        for i in range(len(parser_score_lines)):
            #New test point, both files will share this line. 
            if parser_score_lines[i].startswith('#'):
                #Add the previous data point if we have it. 
                if not target == None:
                    #Get scores in lists so that we may interpolate. 
                    speech_scores = process_speech_scores([hypothesis[1] for hypothesis in hypotheses])
                    parse_scores = process_parse_scores([hypothesis[3] for hypothesis in hypotheses], [hypothesis[0] for hypothesis in hypotheses], parser)
                    lm_scores = [hypothesis[4] for hypothesis in hypotheses]

                    #First interpolate parser and LM scores. 
                    interpolated_scores, parse_scores, lm_scores = interpolate_score_lists(parse_scores, lm_scores, alpha)

                    """
                    print '****************'

                    for i in range(len(hypotheses)):
                        print [hypotheses[i][0], hypotheses[i][2], parse_scores[i], lm_scores[i]]

                    print '***************'
                    """

                    #Next interpolate this score with the speech score. 
                    interpolated_scores, _, _ = interpolate_score_lists(speech_scores, interpolated_scores, beta)

                    #Now create hypothesis list with the interpolated scores. 
                    new_hyp_list = []

                    for j in range(len(hypotheses)):
                        #We just need the phrase and semantic form from this hypothesis. 
                        phrase, _, sem_form, _, _, time = hypotheses[j]
                        new_hyp_list.append([phrase, None, sem_form, interpolated_scores[j], time])

                    #Now re-rank the list. 
                    re_ranked_list = sorted(new_hyp_list, key=lambda x: x[3], reverse=True)
                    
                    # TODO Uncomment to get only top-ranked hypothesis. 
                    re_ranked_list = [re_ranked_list[0]]

                    results.append((target, re_ranked_list))

                    #Keep track of the un-reranked results. 
                    unranked_results.append((target, new_hyp_list[0]))

                target = parser_score_lines[i].strip().split(';')

                #New hypothesis list for this target. 
                hypotheses = []
            else:
                #TODO use time if we want to incorporate it at some point. 
                phrase, speech_score, sem_form, parser_score, time = parser_score_lines[i].strip().split(';')
                _, _, _, lm_score = lm_score_lines[i].strip().split(';')

                #Prepare hypothesis for interpolated re-ranking. 
                hypothesis = [phrase, float(speech_score), sem_form, float(parser_score), float(lm_score), float(time)]
                hypotheses.append(hypothesis)

        assert(len(unranked_results) == len(results))

        """
        for i in range(len(unranked_results)):
            unranked_acc = evaluate.eval_google_full_sem_form([unranked_results[i]], parser)
            ranked_acc = evaluate.eval_google_full_sem_form([results[i]], parser)

            unranked_wer = evaluate.eval_google_wer([unranked_results[i]], parser)
            ranked_wer = evaluate.eval_google_wer([results[i]], parser)

            if ranked_acc > unranked_acc and ranked_wer < unranked_wer: 
                print results[i]
                print ''
                print unranked_results[i]

                print [unranked_wer, unranked_acc, ranked_wer, ranked_acc]

        sys.exit()
        """

        #Now evaluate results for this fold.
        timeout_failure_prop, timeout_total_prop = timeout_proportion(results, parser)
        timeouts_failures.append(timeout_failure_prop)
        timeouts_total.append(timeout_total_prop)

        acc_scores.append(evaluate.eval_google_full_sem_form(results, parser))
        f1_scores.append(evaluate.eval_google_partial_sem_form(results, parser))
        wer_scores.append(evaluate.eval_google_wer(results, parser))

    print ('ACC: ' + str(acc_scores))
    print ('WER: ' + str(wer_scores))
    print ('F1: ' + str(f1_scores))
    print ('TIMEOUT proportions within failures: ' + str(timeouts_failures))
    print ('TIMEOUT proportions total: ' + str(timeouts_total))

    f1_avg = float(sum(f1_scores)) / float(len(f1_scores))
    acc_avg = float(sum(acc_scores)) / float(len(acc_scores))
    wer_avg = float(sum(wer_scores)) / float(len(wer_scores))

    timeout_fail_avg = float(sum(timeouts_failures)) / float(len(timeouts_failures))
    timeout_total_avg = float(sum(timeouts_total)) / float(len(timeouts_total))

    print ('TIMEOUT FAILURE AVG: ' + str(timeout_fail_avg))
    print ('TIMEOUT TOTAL AVG: ' + str(timeout_total_avg))

    return [f1_avg, acc_avg, wer_avg]

def evaluate_file_with_interpolation(experiment_folder, evaluation_name, parser_params, lm_params, alpha, beta): 
    #Read in all the necessary information we'll need for processing. 
    parsers = {}
    parser_score_lines_files = {}
    lm_score_lines_files = {}

    folds = os.listdir(experiment_folder)

    for fold in folds: 
        #Derive paths for files we need. 
        results_path = experiment_folder + fold + '/experiments/asr/result_files/'
        parser_score_file_path = results_path + parser_params + '.' + evaluation_name 
        parser_score_file_path = parser_score_file_path + '_rerank' if 'validation' in evaluation_name else parser_score_file_path
        lm_score_file_path = results_path + lm_params + '.' + evaluation_name + '_lm'

        #Load parser for fold. 
        parser_path = experiment_folder + fold + '/models/' + parser_params + '.cky'
        parsers[fold] = load_obj_general(parser_path)

        #Read in lines from each file for processing.
        parser_score_lines_files[fold] = [line for line in open(parser_score_file_path, 'r')]
        lm_score_lines_files[fold] = [line for line in open(lm_score_file_path, 'r')]

    #Get averages for each evaluation metric using alpha and beta parameters. 
    f1_avg, acc_avg, wer_avg = average_interpolated_scores(folds, parsers, parser_score_lines_files, lm_score_lines_files, alpha, beta) 

    print ([wer_avg, acc_avg, f1_avg])


"""
Test different interpolation values on a validation set
file in order to see what the best interpolation parameters might 
be. 
"""
def find_best_interpolation_values(experiment_folder, evaluation_name, parser_params, lm_params):

    #Read in all the necessary information we'll need for processing. 
    parsers = {}
    parser_score_lines_files = {}
    lm_score_lines_files = {}

    folds = os.listdir(experiment_folder)

    for fold in folds: 
        #Derive paths for files we need. 
        results_path = experiment_folder + fold + '/experiments/asr/result_files/'
        parser_score_file_path = results_path + parser_params + '.' + evaluation_name
        parser_score_file_path = parser_score_file_path + '_rerank' if 'validation' in evaluation_name else parser_score_file_path
        lm_score_file_path = results_path + lm_params + '.' + evaluation_name + '_lm'

        #Load parser for fold. 
        parser_path = experiment_folder + fold + '/models/' + parser_params + '.cky'
        parsers[fold] = load_obj_general(parser_path)

        #Read in lines from each file for processing.
        parser_score_lines_files[fold] = [line for line in open(parser_score_file_path, 'r')]
        lm_score_lines_files[fold] = [line for line in open(lm_score_file_path, 'r')]

    #Values to test for alpha (i.e. interpolation between parser and LM confidences).  
    alphas = np.arange(0.0, 1.1, 0.1)
    betas = np.arange(0.0, 1.1, 0.1) #TODO Change to include a full range once we have implemented the speech surrogate confidence scoring. 

    #Average scores over folds. 
    f1_avgs = {}
    acc_avgs = {}
    wer_avgs = {}

    f1_file = open('f1_tuning.txt', 'w')
    acc_file = open('acc_tuning.txt', 'w')
    wer_file = open('wer_tuning.txt', 'w')

    #Test all possible values. 
    for alpha in alphas:
        for beta in betas:
            print ('Evaluating (alpha, beta): ' + str((alpha, beta)))

            #Now average the scores and assign it to this (alpha, beta) pair. 
            f1_avgs[(alpha, beta)], acc_avgs[(alpha, beta)], wer_avgs[(alpha, beta)] = average_interpolated_scores(folds, parsers, parser_score_lines_files, lm_score_lines_files, alpha, beta) 

    for alpha in alphas: 
        for beta in betas: 
            f1_file.write(str((alpha, beta)) + ';' + str(f1_avgs[(alpha, beta)]) + '\n')
            acc_file.write(str((alpha, beta)) + ';' + str(acc_avgs[(alpha, beta)]) + '\n')
            wer_file.write(str((alpha, beta)) + ';' + str(wer_avgs[(alpha, beta)]) + '\n')

    f1_file.close()
    acc_file.close()
    wer_file.close()

"""
Given a test file, assigns
parse confidence scores 
to all hypotheses produced
by the Google Speech API. 
"""
def give_google_parse_scores(test_file_path, results_file_path, parser_path, google_recognized_folder):      
    
    #First see where we left off if we have already scored parts of this file. 
    test_index_chkpt = 0
    hyp_index_chkpt = 0

    if os.path.exists(results_file_path):
        result_file = open(results_file_path, 'r+')

        #Check takes the form of determing both test line index and hypothesis line index. 
        for line in result_file: 
            if line.startswith('#'):
                hyp_index_chkpt = 0
                test_index_chkpt += 1
            else: 
                hyp_index_chkpt += 1

        result_file.close()

    print ('Starting at test example #' + str(test_index_chkpt) + ' hypothesis #' + str(hyp_index_chkpt))

    #Get lines from test file and open result file for, but now for writing. 
    test_file = open(test_file_path, 'r')
    result_file = open(results_file_path, 'a+')

    parser = load_obj_general(parser_path)
    compute_parser_null_node_values(parser)

    #Keep track of test line indexes, compare to checkpoint indexes s.t. we don't repeat any work. 
    test_index = 1

    for line in test_file:

        #Don't score line we've scored in the past. 
        if test_index >= test_index_chkpt:
            
            #Get components of phrase. 
            phrase, sem_form, denotation, rec = line.strip().split(';')

            #Make sure we don't already have ground truth written. 
            if not (test_index == test_index_chkpt):
                print ('#' + line.strip())
                result_file.write('#' + line)
                result_file.flush()

            #Get name of recording to open google recognition results file. 
            rec_name = os.path.splitext(os.path.basename(rec))[0]
            
            #Get nbest file. 
            nbest_file = open(google_recognized_folder + rec_name + '.nbest', 'r')

            hyp_index = 1

            print (google_recognized_folder + rec_name + '.nbest')

            for result_line in nbest_file:

                #Make sure we haven't already scored this hypothesis. 
                if not (test_index == test_index_chkpt and hyp_index <= hyp_index_chkpt): 
                    hypothesis, google_hyp_score = result_line.strip().split(';')
                    
                    #Format google hypothesis for evaluation. 
                    hypothesis = hypothesis.replace('.', '').lower()

                    #Tokenize so we can parse. 
                    tokenized_hyp = tokenize_for_parser(hypothesis)
                    num_toks = len(tokenized_hyp.split())

                    #Some times parser suffers an error.

                    #Only allow for top 10 hypotheses in order to reduce computation time.
                    if hyp_index <= 10:
                        try:
                            #print 'Attempting to parse: ' + tokenized_hyp

                            #Gets parse.
                            start_time = time.time()
                            parse = timed_get_first_valid_parse(tokenized_hyp, parser, 10)
                            end_time = time.time()

                            #Compute duration of parse for statistics. 
                            duration = end_time - start_time

                            if not parse[0] == None:
                                #Computes avg. production probability. 
                                avg_prod_prob = float('-inf')
                                num_prods = 0.0
                                productions = count_ccg_productions(parse[0])

                                for production in productions:
                                    avg_prod_prob = np.logaddexp(avg_prod_prob, parser.theta.CCG_production[production])
                                    num_prods += float(productions[production])

                                avg_prod_prob -= np.log(num_prods)

                                #Modifies score according to number of null nodes (max num tokens in corpus is 16). 
                                num_nulls = 16 - num_toks
                                parse_score = parse[1] + num_nulls * (parser.theta.null_prior + avg_prod_prob)

                                parse = (parser.print_parse(parse[0].node), parse_score)                     
                            else:
                                parse = ('None', float('-inf'))

                        except KeyboardInterrupt:
                            raise KeyboardInterrupt

                        except:
                            parse = ('None', float('-inf')) 

                        #Now write speech hyp; speech score; parse hyp; parse score.
                        result_line = hypothesis + ';' + google_hyp_score + ';' + str(parse[0]) + ';' + str(parse[1]) + ';' + str(duration)
                        print (result_line)
    
                        result_file.write(result_line + '\n')
                        result_file.flush()

                        #print 'Done test line ' + str(test_index) + ' hyp ' + str(hyp_index)

                hyp_index += 1

        test_index += 1

    result_file.close()
    test_file.close()

    print ('FINISHED SCORING')

def give_experiments_lm_scores(experiment_path, test_file_name, lm_name, google_recognized_folder):
    pids = []

    #Each fold will have a thread scoring files. 
    for fold in os.listdir(experiment_path):

        pid = 0#os.fork()

        if pid == 0: 
            test_file_path = experiment_path + fold + '/experiments/asr/test_files/'+ test_file_name + '.txt'
            result_file_path = experiment_path + fold + '/experiments/asr/result_files/' + lm_name + '.' + test_file_name + '_lm' 
            lm_path = experiment_path + fold + '/models/language_models/' + lm_name + '.lm'

            #Now give lm scores to this file.
            print ('LM scoring ' + result_file_path)
            give_google_lm_scores(test_file_path, google_recognized_folder, result_file_path, lm_path, fold) 

        else:
            pids.append(pid)

    for pid in pids:
        os.waitpid(pid, 0)

"""
Assigns lm probability scores to all google 
speech hypotheses in a given file which 
has already been processed by the parser. 
"""
def give_google_lm_scores(test_file_path, google_recognized_folder, result_file_path, lm_path, unique_name=''): 

    test_file_lines = [line for line in open(test_file_path, 'r')]
    results_file = open(result_file_path, 'w')

    #Num max tokens. 
    num_max_tokens = 16
    
    for line in test_file_lines:

            #First write ground truth to our result file. 
            results_file.write('#' + line)

            #Get components of phrase. 
            phrase, sem_form, denotation, rec = line.strip().split(';')

            #Get name of recording to open google recognition results file. 
            rec_name = os.path.splitext(os.path.basename(rec))[0]
            
            #Get nbest file. 
            nbest_file_lines = [line for line in open(google_recognized_folder + rec_name + '.nbest', 'r')]

            #For now, keep results to 10. 
            num_done = 0
            max_done = 10

            #Now give a probability score to each line in the nbest file form google. 
            for line in nbest_file_lines:
                    
                if num_done < max_done: 

                    num_done += 1    

                    #Get hypothesis and its confidence score. 
                    phrase, google_score = line.strip().split(';')
                        
                    #Pad phrase with UNK so that we can have comparable scores independent of phrase lenght. 
                    tokenized_phrase = phrase.split()
                    padded_phrase = ' '.join(tokenized_phrase + ['<unk>'] * (num_max_tokens - len(tokenized_phrase)))
                    
                    #Process s.t. alphabet matches our corpus. 
                    padded_phrase = padded_phrase.lower().replace('&', 'and').replace('.', '')

                    #Phrase file to give to probability scorer. 
                    phrase_file = open(unique_name + 'phrase.txt', 'w')
                    phrase_file.write(padded_phrase)
                    phrase_file.close()

                    #Prepares arguments  for runnin probability scoring. 
                    args = ['../bin/SRILM/bin/i686-m64/ngram', '-lm', lm_path, '-ppl', unique_name + 'phrase.txt']

                    #Now score the phrase.
                    prob_file = open(unique_name + 'prob.txt', 'w')
                    subprocess.call(args, stdout=prob_file, stderr=prob_file)
                    prob_file.close()

                    #Now get the probability score from it. 
                    prob_file_lines = open(unique_name + 'prob.txt', 'r').read().split('\n')

                    for line in prob_file_lines:
                        if 'logprob' in line:
                            lm_score = float(line.split('logprob= ')[1].split(' ppl')[0])

                    #Write line updated with language model score.
                    results_file.write(';'.join([phrase, google_score, phrase, str(lm_score)]) + '\n')

    #Close the newly written results file. 
    results_file.close()


"""
This method takes a file with n results
from speech recognition and re-ranks them
using the CKYParser class. 
"""
def re_rank_CKY(nbest_file_name, re_ranked_file_name, parser_path, temp_name=''):
    nbest_file = open(nbest_file_name, 'r')
    new_nbest_file = open(temp_name + 'temp.txt', 'w')  #Will be same as nbest, but contain parses so that it may be evaluated using the semantic form evaluations. 

    #Loads parser. 
    parser = load_obj_general(parser_path)

    #Computes values needed for null node contribution.
    compute_parser_null_node_values(parser)

    #Keeps track of data. 
    data = []
    hypotheses = None
    ground_truth = None
    parse = None

    #Limits the number of hypotheses to re-rank.
    limit = 10

    for line in nbest_file:
        #Delimits new phrase. 
        if line.startswith('#'):
            counter = 0

            #Adds last phrase and hypotheses if it exists. 
            if not (hypotheses == None or ground_truth == None or parse == None):
                data.append([ground_truth, hypotheses])

            ground_truth = line.strip().split('#')[1]
            
            #Starts new list of hypotheses. 
            hypotheses = []

            new_nbest_file.write(line)
        else:
            counter += 1
    
            #Gets hypothesis and feeds it to parser to get top scoring parse score. 
            hypothesis = line.strip().split(';')[0]
            tokenized_hypothesis = tokenize_for_parser(hypothesis)
            num_toks = len(tokenized_hypothesis.split())

            #Hypotheses greater than 7 tokens in length are not considered. 
            if counter > limit or num_toks > 7:
                parse = (None, float('-inf'))
            else:
                #Some times parser suffers an error. 
                try:
                    #Gets parse. 
                    parse = get_first_valid_parse(tokenized_hypothesis, parser)

                    if not parse[0] == None:
                        #Computes avg. production probability. 
                        avg_prod_prob = float('-inf')
                        num_prods = 0.0
                        productions = count_ccg_productions(parse[0])

                        for production in productions:
                            avg_prod_prob = np.logaddexp(avg_prod_prob, parser.theta.CCG_production[production])
                            num_prods += float(productions[production])

                        avg_prod_prob -= np.log(num_prods)

                        #Modifies score according to number of null nodes. 
                        num_nulls = 7 - num_toks
                        parse_score = parse[1] + num_nulls * (parser.theta.null_prior + avg_prod_prob)

                        parse = (parse[0], parse_score)

                except TypeError:
                    parse = (None, float('-inf'))
                    
            hypotheses.append([hypothesis, parse])

            #Writes to new nbest file. 
            line_elements = line.strip().split(';')

            #Hypothesis string.
            new_line_elements = [line_elements[0]] 

            #The parse's semantic form.
            if parse[0] == None:
                new_line_elements.append(str(None))
            else:
                new_line_elements.append(parser.print_parse(parse[0].node))  
            
            new_line_elements.append(str(parse[1]))    #The parse's score. 
            new_line_elements.append(line_elements[1]) #The speech hypothesis ASR score.

            new_nbest_file.write(';'.join(new_line_elements) + '\n')

    #Gets the last hypothesis. 
    if not (hypotheses == None or ground_truth == None or parse == None):
        data.append([ground_truth, hypotheses])

    #Reranks list.
    for truth_hyp_list in data:
        truth_hyp_list[1] = sorted(truth_hyp_list[1], key=get_hyp_score, reverse=True)
        
    #Writes re-ranked hypotheses to new file. 
    re_ranked_file = open(re_ranked_file_name, 'w')
    
    for truth_hyp_list in data:
        #Writes ground truth phrase. 
        re_ranked_file.write('#' + truth_hyp_list[0] + '\n')

        #Writes hypotheses. 
        for hypothesis in truth_hyp_list[1]:
            #The hypothesis phrase string itself.
            hyp_str = hypothesis[0] + ';'                            
            
            #The hypothesis' parse semantic form. 
            if not hypothesis[1][0] == None:
                hyp_str += parser.print_parse(hypothesis[1][0].node) + ';'
            else:
                hyp_str += "None;"

            #The parse's score. 
            hyp_str += str(hypothesis[1][1]) + '\n'

            re_ranked_file.write(hyp_str)

    re_ranked_file.close()

    #Updates n-best file. 
    os.rename(temp_name + 'temp.txt', nbest_file_name)

"""
Parses ground truth ASR transcript
in a specific test file. 
"""
def parse_ground_truth_file(test_file_path, result_file_path, parser_path):
    #Load parser. 
    parser = load_obj_general(parser_path)

    #Open necessary files for running experiment. 
    test_file = open(test_file_path, 'r')
    result_file = open(result_file_path, 'w')

    for line in test_file:
        print ('parsing: '  + line.split(';')[0])
        phrase, true_sem_form, _, _ = line.split(';')
        parse = timed_get_first_valid_parse(tokenize_for_parser(phrase), parser, 10)[0]

        #Now convert parse to string. 
        if not parse == None:
            parse_str = parser.print_parse(parse.node)
        else:
            parse_str = "None"

        #Take away CCG category from sem form. 
        true_sem_form = ':'.join(true_sem_form.strip().split(':')[1:])

        # Create result lines with dummy values for unneeded fields.
        ground_truth_line = '#_;M : ' + true_sem_form + ';_;_\n'
        result_line = '_;0.0;' + parse_str + ';1.0;10.0\n' 
        
        #Write result. 
        result_file.write(ground_truth_line + result_line)

"""
Runs the parser on the ground truth
speech transcriptions in order to check the upper 
bound in parsing performance. Creates a result file
for each fold within the specified results_folder. 
"""
def parse_ground_truth(experiment_dir, results_folder, test_file): 
    for fold_name in os.listdir(experiment_dir):
        pid = os.fork()

        if pid == 0: 
            #Loads parser for fold. 
            parser_path = experiment_dir + fold_name + '/models/parser.cky'
            parser = load_obj_general(parser_path)

            #Will keep parsing results. 
            result_file = open(results_folder + fold_name + '.txt', 'w')

            #Parses from validation set. 
            validation_file = experiment_dir + fold_name + '/experiments/asr/test_files/' + test_file
            parse_ground_truth_file(validation_file, result_file, parser)

            result_file.close()

            #Child exit. 
            sys.exit()

#########################################################################################################


"""
Prints the usage for all the functions
conducting ASR experiments, such as getting
the n-best hypothesis from Spinx, re-ranking,
etc. 
"""
def print_usage():
    print ('ASR Nbest: ./experiments asr_n_best [sphinx_shared_library] [ac_model] [lm] [dict] [test_file] [nbest_file] [n]')
    print ('CKYParser re-rank: ./experiments parser_rerank [nbest_file] [re-ranked_file_name] [parser_path] [temp_name] [optional: null_node]')
    print ('Re-rank with interpolation: ./experiments rerank_interpolation [nbest_file] [out_file] [weight]')
    print ('Run parser on speech ground truth: ./experiments parse_ground_truth [test_file] [results_file] [parser_path]')
    print ('Assign parse scores to Google Speech API files: ./experiments parse_score_google [test_file_path] [results_file_path] [parser_path] [google_recognized_folder]')
    print ('Assign probability scores to Google Speech using an LM: ./experiments lm_score_google [experiment_path] [test_file_name] [lm_name] [google_recognized_folder]')
    print ('Find best interpolation values over Speech, Parser, and LM: ./experiments best_interpolation_values [experiment_path] [test_file_name] [parser_params] [lm_params]')
    print ('Evaluate a file using interpolation: ./experiments evaluate_with_interpolation [experiment_path] [test_file_name] [parser_params] [lm_params] [alpha] [beta]')

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'asr_n_best':
        if not len(sys.argv) == 9:
            print_usage()
        else:
            sphinx = SphinxExperiment(sys.argv[2])
            sphinx.init_decoder(sys.argv[3], sys.argv[4], sys.argv[5])
            sphinx.n_best(sys.argv[6], sys.argv[7], sys.argv[8])
            sphinx.close_decoder()

    elif sys.argv[1] == 'rerank_interpolation':
        if len(sys.argv) == 5:
            re_rank_with_interpolation(sys.argv[2], sys.argv[3], float(sys.argv[4]))
        else:
            print_usage()

    elif sys.argv[1] == 'parse_ground_truth':
        if len(sys.argv) == 5:
            parse_ground_truth_file(sys.argv[2], sys.argv[3], sys.argv[4])
        else:
            print_usage()

    elif sys.argv[1] == 'parser_rerank':
        if len(sys.argv) == 6:
            re_rank_CKY(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        elif len(sys.argv) == 7:
            re_rank_function(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])

    elif sys.argv[1] == 'parse_score_google':
        if len(sys.argv) == 6: 
            give_google_parse_scores(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        else:
            print_usage()

    elif sys.argv[1] == 'lm_score_google': 
        if len(sys.argv) == 6: 
            give_experiments_lm_scores(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        else: 
            print_usage()

    elif sys.argv[1] == 'best_interpolation_values':
        if len(sys.argv) == 6:
            find_best_interpolation_values(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        else:
            print_usage()

    elif sys.argv[1] == 'evaluate_with_interpolation':
        if len(sys.argv) == 8: 
            evaluate_file_with_interpolation(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], float(sys.argv[6]), float(sys.argv[7])) 
        else:
            print_usage()

    else:
        print_usage()
