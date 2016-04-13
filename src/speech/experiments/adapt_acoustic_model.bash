#!/bin/bash
#This file adapts the default acoustic model with given training data. 

IDS_FILE=$1 		#File with transcript file ids. (i.e. user_recording_1.raw)
AC_MODEL=$2			#Directory for sphinx en-us acoustic model. 
DICT=$3				#Path for pronunciation dictionary file. 
TRANSCRIPT=$4		#File with transcription for each adaptation utterance. 
RECORDINGS_DIR=$5	#Directory where the adaptation recordings are to be found. 
OUT_DIR=$6			#Directory where all intermediary files (e.g. feature files, etc.) for adaptation process are kept.
ADAPT_MODEL=$7		#Directory where adapted acoustic model will be kept. 
BIN=$8				#Path for binary executables used by adaptation process.

CURRENT_DIR=$( pwd )
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CURRENT_DIR/sphinxbase_install/lib/

#Creates feature files for each recording. 
$CURRENT_DIR/bin/sphinx_fe -argfile $AC_MODEL/feat.params -samprate 16000 -c $IDS_FILE -di $RECORDINGS_DIR -do $OUT_DIR -ei raw -eo mfc -raw yes

#Accumulates observation counts. 
$BIN/bw -hmmdir $AC_MODEL -moddeffn $AC_MODEL/mdef -ts2cbfn .ptm. -feat 1s_c_d_dd -svspec 0-12/13-25/26-38 -cmn current -agc none -dictfn $DICT -ctlfn $IDS_FILE -lsnfn $TRANSCRIPT -accumdir $OUT_DIR -cepdir $OUT_DIR

#Makes new directory for adapted model. 
cp -a $AC_MODEL $ADAPT_MODEL

#Uses MAP to adapt model. 
$BIN/map_adapt -moddeffn $AC_MODEL/mdef -ts2cbfn .ptm. -meanfn $AC_MODEL/means -varfn $AC_MODEL/variances -mixwfn $AC_MODEL/mixture_weights -tmatfn $AC_MODEL/transition_matrices -accumdir $OUT_DIR -mapmeanfn $ADAPT_MODEL/means -mapvarfn $ADAPT_MODEL/variances -mapmixwfn $ADAPT_MODEL/mixture_weights -maptmatfn $ADAPT_MODEL/transition_matrices
