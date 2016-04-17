#!/bin/bash

CURRENT_DIR=$( pwd )

for i in $( ls $CURRENT_DIR/k-folds ); do
	for j in $( ls $CURRENT_DIR/k-folds/$i ); do
		python gen_small_parser_file.py $CURRENT_DIR/k-folds/$i/$j
	done
done
