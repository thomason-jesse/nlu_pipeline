#!/bin/bash

USER_ID="$1"
HOST="$2"
PORT="$3"

#Makes directories for session. 
mkdir -p results/headset/$USER_ID/denotations
mkdir -p results/headset/$USER_ID/phrases
mkdir -p results/headset/$USER_ID/recordings
mkdir -p results/headset/$USER_ID/semantic_forms

#Runs recording program with given arguments.
python record.py $USER_ID mix y n $HOST $PORT
