#!/bin/bash
#Concatenates data files for all users in each fold. 
#Also adds recordings for each fold. 

CURRENT_DIR=$( pwd )

for i in $( ls $CURRENT_DIR/corpus ); do
	for j in $( ls $CURRENT_DIR/corpus/$i ); do
		cat $CURRENT_DIR/corpus/$i/$j/*/*_train_lm.txt > $CURRENT_DIR/corpus/$i/$j/"$j"_train_lm.txt;
		cat $CURRENT_DIR/corpus/$i/$j/*/*_train_parser.txt > $CURRENT_DIR/corpus/$i/$j/"$j"_train_parser.txt;
		cat $CURRENT_DIR/corpus/$i/$j/*/*_train_parser_short.txt > $CURRENT_DIR/corpus/$i/$j/"$j"_train_parser_short.txt;
		cat $CURRENT_DIR/corpus/$i/$j/*/*_train_parser_tiny.txt > $CURRENT_DIR/corpus/$i/$j/"$j"_train_parser_tiny.txt;
		cat $CURRENT_DIR/corpus/$i/$j/*/*_recording_files.txt > $CURRENT_DIR/corpus/$i/$j/"$j"_recording_files.txt;
		cat $CURRENT_DIR/corpus/$i/$j/*/*_transcript.txt > $CURRENT_DIR/corpus/$i/$j/"$j"_transcript.txt
	done
done
