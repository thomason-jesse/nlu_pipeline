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

#TODO Change if necessary. 
#Adds nlu_pipeline src folder in order to import modules from it. 
nlu_pipeline_path = '/scratch/cluster/rcorona/nlu_pipeline/src/'
sys.path.append(nlu_pipeline_path)

#Nlu pipeline modules.
try:
    import CKYParser
    from CKYParser import count_ccg_productions
    from utils import *
except ImportError:
    print 'ERROR: Unable to load nlu_pipeline_modules! Verify that nlu_pipeline_path is set correctly!'
    sys.exit()



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

    #Normalizes by |SF| * |CCG|
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

    #Computes p(_ | _, null) i.e. the average probability of a production. 
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
 
    print count_ccg_productions(parser.most_likely_cky_parse("go to ray 's office").next()[0])

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
Re-ranks a file which contains both speech
and semantic parse scores by interpolating them 
using a given value to weight them. 
"""
def re_rank_with_interpolation(in_file_name, out_file_name, weight):
    in_file = open(in_file_name, 'r')
    out_file = open(out_file_name, 'w')

    line = in_file.readline()
    hyp_buff = []

    while not line == '':
        #Denotes beginning of new phrase hypothesis list. 
        if line.startswith('#'):
            #Buffer will be empty before first phrase. 
            if len(hyp_buff) > 0: 
                hyp_buff = re_rank_hypotheses_with_interpolation(hyp_buff, weight)

                #Write each hypothesis which is now re-ranked. 
                for hyp in hyp_buff:
                    out_file.write(hyp + '\n')

                #Clear buffer. 
                hyp_buff = []

            #Now write start of new phrase to out file and get next line. 
            out_file.write(line)

        #Otherwise collect hypothesis for eventual re-ranking. 
        else:
            hyp_buff.append(line.strip())

        #Now get next line. 
        line = in_file.readline()
    
    #Write last phrase results. 
    for hyp in hyp_buff:
        out_file.write(hyp + '\n')

    #Close files and finish. 
    in_file.close()
    out_file.close()

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

    print 'Starting at test example #' + str(test_index_chkpt) + ' hypothesis #' + str(hyp_index_chkpt)

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
                print '#' + line.strip()
                result_file.write('#' + line)
                result_file.flush()

            #Get name of recording to open google recognition results file. 
            rec_name = os.path.splitext(os.path.basename(rec))[0]
            
            #Get nbest file. 
            nbest_file = open(google_recognized_folder + rec_name + '.nbest', 'r')

            hyp_index = 1

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
                        print result_line
    
                        result_file.write(result_line + '\n')
                        result_file.flush()

                        #print 'Done test line ' + str(test_index) + ' hyp ' + str(hyp_index)

                hyp_index += 1

        test_index += 1

    result_file.close()
    test_file.close()

    print 'FINISHED SCORING'

"""
Assigns lm probability scores to all google 
speech hypotheses in a given file which 
has already been processed by the parser. 
"""
def give_google_lm_scores(results_file_path, new_results_file_path, lm_path): 

    results_file = open(results_file_path, 'r')
    new_results_file = open(new_results_file_path, 'w')

    #Num max tokens. 
    num_max_tokens = 16

    for line in results_file: 
        #If ground truth line, then simply re-write it. 
        if line.startswith('#'):
            new_results_file.write(line)
        else: 
            #We only need to the phrase to derive a score. 
            phrase, google_score, sem_form, parser_score, time = line.strip().split(';')

            #Pad phrase with UNK so that we can have comparable scores independent of phrase lenght. 
            tokenized_phrase = phrase.split()
            padded_phrase = ' '.join(tokenized_phrase + (['UNK'] * (num_max_tokens - len(tokenized_phrase))))

            #Phrase file to give to probability scorer. 
            phrase_file = open('phrase.txt', 'w')
            phrase_file.write(padded_phrase)
            phrase_file.close()

            #Prepares arguments  for runnin probability scoring. 
            args = ['../bin/SRILM/bin/i686-m64/ngram', '-lm', lm_path, '-ppl', 'phrase.txt']

            #Now score the phrase.
            prob_file = open('prob.txt', 'w')
            subprocess.call(args, stdout=prob_file, stderr=prob_file)
            prob_file.close()

            #Now get the probability score from it. 
            prob_file = open('prob.txt', 'r')
            lm_score = float(prob_file.read().split('\n')[1].split('logprob= ')[1].split(' ppl')[0])

            #Write line updated with language model score. 
            new_results_file.write(';'.join([phrase, google_score, sem_form, parser_score, time, str(lm_score)]) + '\n')

    #Close the newly written results file. 
    new_results_file.close()

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
        print 'parsing: '  + line.split(';')[0]
        phrase, true_sem_form, _, _ = line.split(';')
        parse = timed_get_first_valid_parse(tokenize_for_parser(phrase), parser, 10)[0]

        #Now convert parse to string. 
        if not parse == None:
            parse_str = parser.print_parse(parse.node)
        else:
            parse_str = "None"

        #Take away CCG category from sem form. 
        true_sem_form = ':'.join(true_sem_form.strip().split(':')[1:])

        #Write result. 
        result_file.write(true_sem_form + ';' + parse_str + '\n')

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
    print 'ASR Nbest: ./experiments asr_n_best [sphinx_shared_library] [ac_model] [lm] [dict] [test_file] [nbest_file] [n]'
    print 'CKYParser re-rank: ./experiments parser_rerank [nbest_file] [re-ranked_file_name] [parser_path] [temp_name] [optional: null_node]'
    print 'Re-rank with interpolation: ./experiments rerank_interpolation [nbest_file] [out_file] [weight]'
    print 'Run parser on speech ground truth: ./experiments parse_ground_truth [test_file] [results_file] [parser_path]'
    print 'Assign parse scores to Google Speech API files: ./experiments parse_score_google [test_file_path] [results_file_path] [parser_path] [google_recognized_folder]'
    print 'Assign probability scores to Google Speech using an LM: ./experiments lm_score_google [result_file_path] [new_results_file_path] [lm_path]'

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
        if len(sys.argv) == 5: 
            give_google_lm_scores(sys.argv[2], sys.argv[3], sys.argv[4])
        else: 
            print_usage()
    else:
        print_usage()
