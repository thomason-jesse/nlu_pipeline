#!/bin/bash
#This script submits condor jobs for generating all the parsers. 

CURRENT_DIR=$( pwd )

for i in $( ls $CURRENT_DIR/k-folds ); do 
	for j in $( ls $CURRENT_DIR/k-folds/$i ); do
		condor_submit -a "CURRENT_DIR = $CURRENT_DIR" -a "NAME = $i-$j-tiny-8" -a "SCRIPT = $CURRENT_DIR/../../train_parser.py" -a "TRAIN = $CURRENT_DIR/k-folds/$i/$j/parser_train_tiny.txt" -a "PARSER = $CURRENT_DIR/k-folds/$i/$j/parser_tiny_8.cky" $CURRENT_DIR/condor/train_parser.bash -a "EXISTING_PARSER = $CURRENT_DIR/k-folds/$i/$j/parser_tiny.cky" 
	done
done
