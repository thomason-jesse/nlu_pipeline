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
    #Currently only splits possessive. 
    return phrase.replace("'", " '")

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
Returns the last element of a list. 
"""
def get_last_element_as_float(l):
    return float(l[-1])

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

    elif sys.argv[1] == 'parser_rerank':
        if len(sys.argv) == 6:
            re_rank_CKY(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        elif len(sys.argv) == 7:
            re_rank_function(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])
