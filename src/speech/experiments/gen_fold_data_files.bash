#!/bin/bash
#Concatenates data files for all users in each fold. 

CURRENT_DIR=$( pwd )

for i in $( ls $CURRENT_DIR/corpus ); do
	for j in $( ls $CURRENT_DIR/corpus/$i ); do
		cat $CURRENT_DIR/corpus/$i/$j/*/*_train_lm.txt > $CURRENT_DIR/corpus/$i/$j/"$j"_train_lm.txt;
		cat $CURRENT_DIR/corpus/$i/$j/*/*_train_parser.txt > $CURRENT_DIR/corpus/$i/$j/"$j"_train_parser.txt;
	done
done
