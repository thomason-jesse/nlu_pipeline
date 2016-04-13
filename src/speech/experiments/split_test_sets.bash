#!/bin/bash
#This script splits the test set in the folds
#in two in preparation for second phase of experiments. 

CURRENT_DIR=$( pwd )

for i in $( ls $CURRENT_DIR/k-folds ); do
	for j in $( ls $CURRENT_DIR/k-folds/$i ); do
		python split_test_set.py $CURRENT_DIR/k-folds/$i/$j/recordings/test/ $i 
	done
done
