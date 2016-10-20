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
            phrase = line.strip().split('#')[1].split(';')[0]

            #Line immediately after phrase is top scoring hypothesis. 
            line = result_file.readline().strip()
            hypothesis = line.split(';')[0]

            data.append([phrase, hypothesis])

    #Creates temporary files to run evaluations on.
    os.chdir('../bin')
    transcript_file = open('transcripts.txt', 'w')
    hyp_file = open('hyp.txt', 'w')

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
    args = [bin_path + '/word_align.pl', 'transcripts.txt', 'hyp.txt']

    #Evaluates WER. 
    subprocess.call(args, stdout=evaluation_file, stderr=evaluation_file)

    #Deletes temp files and closes evaluation. 
    os.remove('transcripts.txt')
    os.remove('hyp.txt')
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
            phrase, semantic_form = line.strip().split('#')[1].split(';')

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


def print_usage():
    print 'WER: ./evaluate.py wer [result_file] [evaluation_file_name]'
    print 'Correct hypothesis in top n: ./evaluate.py top_n [result_file] [evaluation_file_name] [n]'
    print 'Semantic form evaluation: ./evaluate.py semantic_form [result_file] [evaluation_file_name] [parser]'
    print 'Grounding: ./evaluate.py grounding [ont file] [lex file] [kb pickle file]'


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
        if not len(sys.argv) == 5:
            print_usage()
        else:
            semantic_form(sys.argv[2], sys.argv[3], sys.argv[4])

    elif sys.argv[1] == 'grounding':
        if len(sys.argv) == 5:
            grounded_forms(sys.argv[2], sys.argv[3], sys.argv[4])
        else:
            print_usage()

    else:
        print_usage()
