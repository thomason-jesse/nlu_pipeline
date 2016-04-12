#!/bin/bash

#TODO add options to run only certain experiments. 

#Generates corpus directory structure. 
mkdir corpus
mkdir corpus/4_fold
#mkdir corpus/8_fold

#Generates k-folds directory structure. 
mkdir k-folds
mkdir k-folds/4_fold
#mkdir k-folds/8_fold

echo "Generating folds..."
./gen_corpus_folds.bash 4 ../script/recording/results/headset corpus/4_fold k-folds/4_fold
#./gen_corpus_folds.bash 8 ../script/recording/results/headset corpus/8_fold k-folds/8_fold

#Generates the data files for each fold. 
echo "Generating data files for each fold..."
./gen_fold_data_files.bash

#Uses data files from each fold to create each folds train and test sets.
echo "Generating train and test sets for each fold..."
python gen_k_folds.py

#Creates in-domain language models for each.
echo "Generating language models for each fold..."
#./gen_lms.bash ./SRILM/bin/i686-m64/

