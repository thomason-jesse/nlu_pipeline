#!/bin/bash

USER_ID="$1"
HOST="$2"
PORT="$3"

#Makes directories for session. 
mkdir -p results/mic/$USER_ID/denotations
mkdir -p results/mic/$USER_ID/phrases
mkdir -p results/mic/$USER_ID/recordings
mkdir -p results/mic/$USER_ID/semantic_forms

#Runs recording program with given arguments.
python record.py $USER_ID results/mic/$USER_ID y n $HOST $PORT
