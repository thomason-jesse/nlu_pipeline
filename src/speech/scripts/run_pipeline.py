#!/usr/bin/python

"""
This script may be used to run the entire
pipeline in as few steps as possible. 


Author: Rodolfo Corona, rcorona@utexas.edu
"""

#Python imports
import sys
import os
import subprocess

#Speech pipeline imports.
import preprocess
import train
import evaluate

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
            for test_file in os.listdir(test_file_path):    
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
                args.append(fold_name)

                #Log file. 
                log_file = open(fold_path + '/logs/experiments/' + test_name + '.cky', 'w')

                #Runs  experiment. 
                subprocess.call(args, stdout=log_file, stderr=log_file)

            #Now evaluates files. 
            for result_file in os.listdir(result_file_path):
                result_file_name = result_file_path + result_file
                eval_path = fold_path + '/evaluations'
                parser_path = fold_path + '/models/parser.cky'

                evaluate.semantic_form(result_file_name, eval_path + '/sem_full/' + result_file, parser_path)
                evaluate.semantic_form(result_file_name, eval_path + '/sem_partial/' + result_file, parser_path)
                evaluate.wer(result_file_name, eval_path + '/wer/' + result_file)
                evaluate.correct_in_top_n(result_file_name, eval_path + '/top_1/' + result_file, 1)
                evaluate.correct_in_top_n(result_file_name, eval_path + '/top_5/' + result_file, 5)

            #Child process exit. 
            sys.exit()



"""
Prints the usage instructions  
for the script. 
"""
def print_usage():
    print 'Create n-fold experiment folder with all files needed for training: ./run_pipeline.py set_up [preprocessed_corpus_path] [n_fold_dir_path] [n]'
    print 'Train models: ./run_pipeline.py train [experiment_dir]'
    print 'Run 1st experiments (re-ranking on initial speech output): ./run_pipeline.py exp1 [experiment_dir]'

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

        else: 
            print_usage()
    else:
        print_usage()

