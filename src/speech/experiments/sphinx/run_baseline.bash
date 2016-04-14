#This script submits all condor jobs to run
#baseline experiments.

CURRENT_DIR=/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments

for i in $( ls $CURRENT_DIR/k-folds ); do
	for j in $( ls $CURRENT_DIR/k-folds/$i ); do
		#Submits jobs for each test set. 
		for num in $( echo 1 2 ); do
			#Submits jobs for weights. 
			for w in $( echo 0.25 0.5 0.75 ); do
				condor_submit -a "NAME = $i-$j-$num-$w" -a "AC_MODEL = $CURRENT_DIR/k-folds/$i/$j/recordings/train/en-us-adapt" -a "DICT = $CURRENT_DIR/resources/cmudict-en-us.dict" -a "LM = $CURRENT_DIR/k-folds/$i/$j/$w.lm" -a "RESULTS_DIR = $CURRENT_DIR/k-folds/$i/$j/decoding/baseline/$num-$w" -a "RECORDINGS_DIR = $CURRENT_DIR/k-folds/$i/$j/recordings/test" -a "IDS_FILE = ids$num.fileids" run_sphinx.bash
			done

			condor_submit -a "NAME = $i-$j-$num-en-us" -a "AC_MODEL = $CURRENT_DIR/k-folds/$i/$j/recordings/train/en-us-adapt" -a "DICT = $CURRENT_DIR/resources/cmudict-en-us.dict" -a "LM = $CURRENT_DIR/resources/en-us.lm" -a "RESULTS_DIR = $CURRENT_DIR/k-folds/$i/$j/decoding/baseline/$num-en-us" -a "RECORDINGS_DIR = $CURRENT_DIR/k-folds/$i/$j/recordings/test" -a "IDS_FILE = ids$num.fileids" run_sphinx.bash

			condor_submit -a "NAME = $i-$j-$num-in-domain" -a "AC_MODEL = $CURRENT_DIR/k-folds/$i/$j/recordings/train/en-us-adapt" -a "DICT = $CURRENT_DIR/resources/cmudict-en-us.dict" -a "LM = $CURRENT_DIR/k-folds/$i/$j/in_domain.lm" -a "RESULTS_DIR = $CURRENT_DIR/k-folds/$i/$j/decoding/baseline/$num-in-domain" -a "RECORDINGS_DIR = $CURRENT_DIR/k-folds/$i/$j/recordings/test" -a "IDS_FILE = ids$num.fileids" run_sphinx.bash

		done
	done
done
