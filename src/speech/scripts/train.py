#!/usr/bin/python

"""
This script contains all functions
relevant to training the models used
on the speech corpus (i.e. parser, 
and asr training). 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

__author__ = ['rcorona', 'aishwarya']

import sys
import subprocess
import shutil
import time

#Adds path to parser scripts so that they may be imported.
#TODO CHANGE IF PATH CHANGES
sys.path.append('/scratch/cluster/rcorona/nlu_pipeline/src/')

#Nlu pipeline modules.
try:
    import Ontology
    import Lexicon
    import CKYParser
    from utils import *
except ImportError as e:
    print 'ERROR: Unable to load nlu_pipeline_modules! Verify that nlu_pipeline_path is set correctly!'
    print e
    sys.exit()

"""
Trains a new semantic parser using
the given training file and saves it
to the given parser_path using
a given ontology and lexicon. 
"""
def train_new_parser(parser_train_file, parser_path, ont_path, lex_path, lex_weight):
    #Converts to ensure floating point and not string. 
    if lex_weight == 'sys.maxint':
        lex_weight = sys.maxint
    else:
        lex_weight = float(lex_weight)

    #Instantiates ontology. 
    print "reading in Ontology"
    ont = Ontology.Ontology(ont_path)
    print "predicates: " + str(ont.preds)
    print "types: " + str(ont.types)
    print "entries: " + str(ont.entries)

    #Instantiates lexicon. 
    print "reading in Lexicon"
    lex = Lexicon.Lexicon(ont, lex_path)
    print "surface forms: " + str(lex.surface_forms)
    print "categories: " + str(lex.categories)
    print "semantic forms: " + str(lex.semantic_forms)
    print "entries: " + str(lex.entries)
    
    #Used to train new parser. 
    parser = CKYParser.CKYParser(ont, lex, use_language_model=False, lexicon_weight=lex_weight)
    
    #Used to train existing parser on new data. 
    #parser = load_obj_general(sys.argv[5])

    # Set parser hyperparams to best known values for test time
    parser.max_multiword_expression = 4  # max span of a multi-word expression to be considered during tokenization
    parser.max_new_senses_per_utterance = 0  # max number of new word senses that can be induced on a training example
    parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
    parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried
    parser.max_expansions_per_non_terminal = 10  # number of backpointers stored per nonterminal per cell in CKY chart

    # Read in train data. 
    data = parser.read_in_paired_utterance_semantics(parser_train_file)

    converged = parser.train_learner_on_semantic_forms(data, 20, reranker_beam=1)
    print '\n\n\nHand designed examples: converged = ', converged

    #Saves trained parser. 
    save_obj_general(parser, parser_path)

"""
Trains a new language model using
the given training file and saves
it to the given lm_path. 
"""
def train_new_lm(lm_train_file, lm_path):
    #Path to executable used to create language model. TODO Change if needed.
    srilm_path = '../bin/SRILM/bin/i686-m64/ngram-count'
    
    #Arguments to call it. 
    args = [srilm_path, '-wbdiscount', '-interpolate', '-text', lm_train_file, '-lm', lm_path]

    #Calls it to create lm. 
    subprocess.call(args)

def adapt_accoustic_model(acoustic_model_path, corpus_folder, recordings_dir, dict_path):
    #Paths needed for execution. 
    bin_path = '../bin'
    sphinxbase_lib_path = '../resources/sphinxbase_install/lib/'

    #Sets LD_LIBRARY_PATH for needed libraries. 
    os.environ['LD_LIBRARY_PATH'] = '$LD_LIBRARY_PATH:' + sphinxbase_lib_path

    #Composes arguments for feature extraction. 
    args = [bin_path + '/sphinx_fe']
    args.append('-argfile')
    args.append(acoustic_model_path + '/feat.params')
    args.append('-samprate')
    args.append('16000')
    args.append('-c')
    args.append(corpus_folder + '/asr_training/train.fileids')
    args.append('-di')
    args.append(recordings_dir)
    args.append('-do')
    args.append(corpus_folder + '/models/ac_model_adapt_files')
    args.append('-ei')
    args.append('raw')
    args.append('-eo')
    args.append('mfc')
    args.append('-raw')
    args.append('yes')

    #Extracts features from adaptation recordings. 
    subprocess.call(args)

    #Accumulates observation counts. 
    args = [bin_path + '/bw']
    args.append('-hmmdir')
    args.append(acoustic_model_path)
    args.append('-moddeffn')
    args.append(acoustic_model_path + '/mdef')
    args.append('-ts2cbfn')
    args.append('.ptm.')
    args.append('-feat')
    args.append('1s_c_d_dd')
    args.append('-svspec')
    args.append('0-12/13-25/26-38')
    args.append('-cmn')
    args.append('current')
    args.append('-agc')
    args.append('none')
    args.append('-dictfn')
    args.append(dict_path)
    args.append('-ctlfn')
    args.append(corpus_folder + '/asr_training/train.fileids')
    args.append('-lsnfn')
    args.append(corpus_folder + '/asr_training/train.transcription')
    args.append('-accumdir')
    args.append(corpus_folder + '/models/ac_model_adapt_files')
    args.append('-cepdir')
    args.append(corpus_folder + '/models/ac_model_adapt_files')

    #Calls executable to accumulate counts. 
    subprocess.call(args)

    #Copies acoustic model folder to new folder. This folder will now be adapted. 
    #If directory exists from previous run, it's deleted before creating a new folder copy. 
    if os.path.isdir(corpus_folder + '/models/adapted_ac_model'):
        shutil.rmtree(corpus_folder + '/models/adapted_ac_model')

    shutil.copytree(acoustic_model_path, corpus_folder + '/models/adapted_ac_model')

    #Sets path for adapted acoustic model. 
    adapted_ac_model = corpus_folder + '/models/adapted_ac_model'

    #Uses MAP algorithm to adapt model.
    args = [bin_path + '/map_adapt']
    args.append('-moddeffn')
    args.append(acoustic_model_path + '/mdef')
    args.append('-ts2cbfn')
    args.append('.ptm.')
    args.append('-meanfn')
    args.append(acoustic_model_path + '/means')
    args.append('-varfn')
    args.append(acoustic_model_path + '/variances')
    args.append('-mixwfn')
    args.append(acoustic_model_path + '/mixture_weights')
    args.append('-tmatfn')
    args.append(acoustic_model_path + '/transition_matrices')
    args.append('-accumdir')
    args.append(corpus_folder + '/models/ac_model_adapt_files')
    args.append('-mapmeanfn')
    args.append(adapted_ac_model + '/means')
    args.append('-mapvarfn')
    args.append(adapted_ac_model + '/variances')
    args.append('-mapmixwfn')
    args.append(adapted_ac_model + '/mixture_weights')
    args.append('-maptmatfn')
    args.append(adapted_ac_model + '/transition_matrices')

    #Runs executable to adapt model.
    subprocess.call(args)


def print_usage():
    print 'To train new parser: ./train.py new_parser [parser_train_file] [parser_save_path] [ont_path] [lex_path] [lex_weight]'
    print 'To train new language model: ./train.py new_lm [lm_train_file] [lm_save_path]'
    print 'To adapt an accoustic model: ./train.py adapt_ac_model [accoustic_model_path] [corpus_folder] [recordings_dir] [dict_path]'

if __name__ == '__main__':
    if not len(sys.argv) >= 4:
        print_usage()
    
    #Train new parser case. 
    elif sys.argv[1] == 'new_parser':
        if not len(sys.argv) == 7:
            print_usage()
        else:
            train_new_parser(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])

    #Train a new language model. 
    elif sys.argv[1] == 'new_lm':
        if len(sys.argv) == 4:
            train_new_lm(sys.argv[2], sys.argv[3])
        else:
            print_usage()

    #Adapt an accoustic model. 
    elif sys.argv[1] == 'adapt_ac_model':
        if len(sys.argv) == 6:
            adapt_accoustic_model(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        else:
            print_usage()
