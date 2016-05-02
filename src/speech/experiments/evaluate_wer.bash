#!/bin/bash

CURRENT_DIR=$( pwd )

for i in $( ls $CURRENT_DIR/k-folds/4_fold/ ); do
	FOLD=$CURRENT_DIR/k-folds/4_fold/$i

	#Evaluates WER for first test set results. 
	for j in $( ls -d $CURRENT_DIR/k-folds/4_fold/$i/decoding/baseline/1* ); do

		echo "Evaluating " $j " ..." 
		echo ""

		./bin/word_align.pl --ignore-uttid $FOLD/recordings/test/transcript1.txt $j/results.txt &> $j/wer.txt
	done

	#Evaluates WER for second test set results. 
	for j in $( ls -d $CURRENT_DIR/k-folds/4_fold/$i/decoding/baseline/2* ); do
		BASELINE=$FOLD/decoding/baseline/$j

		echo "Evaluating " $j " ..."
		echo ""

		./bin/word_align.pl --ignore-uttid $FOLD/recordings/test/transcript2.txt $j/results.txt &> $j/wer.txt
	done
done
