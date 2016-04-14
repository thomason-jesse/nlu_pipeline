#!/bin/bash
#This script submits condor jobs for generating all the parsers. 

CURRENT_DIR=$( pwd )

for i in $( ls $CURRENT_DIR/k-folds ); do 
	for j in $( ls $CURRENT_DIR/k-folds/$i ); do
		condor_submit -a "CURRENT_DIR = $CURRENT_DIR" -a "NAME = $i-$j" -a "SCRIPT = $CURRENT_DIR/../../ParserTrainer.py" -a "TRAIN = $CURRENT_DIR/k-folds/$i/$j/parser_train_small.txt" -a "PARSER = $CURRENT_DIR/k-fold/$i/$j/parser.cky" $CURRENT_DIR/condor/train_parser.bash
	done
done
