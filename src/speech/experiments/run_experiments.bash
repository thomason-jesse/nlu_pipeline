#!/bin/bash

#TODO add options to run only certain experiments. 

#Generates k-fold directory structure. 
mkdir k-folds
mkdir k-folds/4_fold
mkdir k-folds/8_fold

./gen_k_folds.bash 4 ../script/recording/results/headset k-folds/4_fold
./gen_k_folds.bash 8 ../script/recording/results/headset k-folds/8_fold
