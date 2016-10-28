#!/usr/bin/python

"""
This script may be used to run the entire
pipeline in as few steps as possible. 


Author: Rodolfo Corona, rcorona@utexas.edu
"""

#Python imports
import sys
import os

#Speech pipeline imports.
import preprocess

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
Prints the usage instructions  
for the script. 
"""
def print_usage():
    print 'Create n-fold experiment folder with all files needed for training: ./run_pipeline.py set_up [preprocessed_corpus_path] [n_fold_dir_path] [n]'

if __name__ == '__main__':
    if len(sys.argv) >= 2:
        if sys.argv[1] == 'set_up':
            if len(sys.argv) == 5:
                set_up(sys.argv[2], sys.argv[3], sys.argv[4])
            else:
                print_usage()
        else: 
            print_usage()
    else:
        print_usage()

