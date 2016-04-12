#!/bin/bash

#This file generates k k-fold training and testing sets
#from the speech corpus. 
#Usage: $./gen_k_folds.bash [k] [speech corpus path] [k-fold sets path]

K=$1			#The K in k-folds
SPEECH_PATH=$2	#The path to the actual speech corpus.  
K_SET_PATH=$3	#The path in which to create the symbolic links. 
K_FOLD_PATH=$4	#The path where all the actual data used for experiments will be kept. 

#Used to create symbolic links. 
CURRENT_DIR=$( pwd )

#May update this later to not be hard-coded. 
NUM_SPEECH_USERS=32

#TODO Add checks to ensure that parameters will create an even division. 
FOLD_SIZE=$( expr $NUM_SPEECH_USERS / $K )

#Used to keep track of number of elements in current fold. 
NUM_IN_FOLD=0
FOLD_NUM=1

#Makes necessary folders. 
for i in $( seq $K ); do
	mkdir $K_SET_PATH/$i
	mkdir $K_FOLD_PATH/$i
	mkdir -p $K_FOLD_PATH/$i/recordings/train
	mkdir -p $K_FOLD_PATH/$i/recordings/test
done

#Goes through all users in folder to create folds. 
for i in $( ls $SPEECH_PATH ); do
	#Moves to next fold if enough elements in current one. 
	if [ $NUM_IN_FOLD -eq $FOLD_SIZE ]; then
		NUM_IN_FOLD=0
		FOLD_NUM=$(($FOLD_NUM + 1))
	fi

	#Inserts elemtn into fold using symbolic link.
	ln -s $CURRENT_DIR/$SPEECH_PATH/$i $CURRENT_DIR/$K_SET_PATH/$FOLD_NUM/
	
	#Increments NUM_IN_FOLD
	NUM_IN_FOLD=$(($NUM_IN_FOLD + 1))
done
