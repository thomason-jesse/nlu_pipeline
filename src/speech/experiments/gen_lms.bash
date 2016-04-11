#!/bin/bash
#This script generates all language models for each fold. 

#Path to language modeling toolkit binaries. 
SRILM_PATH=SRILM/bin/i686-m64/
export PATH=$PATH:$SRLIM_PATH

CURRENT_DIR=$( pwd )

for i in $( ls k-folds ); do
	for j in $( ls k-folds/$i ); do
		#Creates in domain language model for fold. 
		IN_LM=$CURRENT_DIR/k-folds/$i/$j/in_domain.lm
		ngram-count -wbdiscount -interpolate -text $CURRENT_DIR/k-folds/$i/$j/lm_train.txt -lm $IN_LM
		echo "Created in domain language model for " $i/$j

		#Creates interpolated language models using the sphinx en-us model. 
		for w in $( echo 0.25 0.5 0.75 ); do
			condor/interpolate_lms.bash -a "SRILM_DIR = $SRILM_PATH" -a "NAME = $i-$j-$w" -a "IN_LM = $IN_LM" -a "GEN_LM = resources/en-us.lm" -a "INT_LM = $CURRENT_DIR/k-folds/$i/$j/$w.lm" -a "w = $w"
			echo "Submited condor job for interpolated lm for " $i/$j/ "with weight" $w
		done
	done
done
