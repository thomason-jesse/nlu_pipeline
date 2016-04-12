#!/bin/bash
#This file adapts the default acoustic model with given training data. 

IDS=ids

#Creates feature files for each recording. 
sphinx_fe -argfile en-us/feat.params -samprate 16000 -c $IDS.fileids -di . -do . -ei raw -eo mfc -raw yes

./bw -hmmdir en-us -moddeffn en-us/mdef -ts2cbfn .ptm. -feat 1s_c_d_dd -svspec 0-12/13-25/26-38 -cmn current -agc none -dictfn cmudict-en-us.dict -ctlfn $IDS.fileids -lsnfn final_transcript.txt -accumdir .


