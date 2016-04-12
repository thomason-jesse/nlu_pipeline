#!/bin/bash
#This script generates all of the acoustic models
#to be used in experiments. 

CURRENT_DIR=$( pwd )
IDS_FILE= #TODO
AC_MODEL=$CURRENT_DIR/resources/en-us
DICT=$CURRENT_DIR/resources/cmudict-en-us.dict
BIN=$CURRENT_DIR/bin

for i in $( ls $CURRENT_DIR/k-folds ); do
	for j in $( ls $CURRENT_DIR/k-folds/$i ); do
		RECORD_DIR=$CURRENT_DIR/k-folds/$i/$j/recordings/train
		./adapt_acoustic_model.bash $RECORD_DIR/ids.fileids $AC_MODEL $DICT $RECORD_DIR/transcript.txt $RECORD_DIR $RECORD_DIR $RECORD_DIR/en-us-adapt $BIN 
	done
done
