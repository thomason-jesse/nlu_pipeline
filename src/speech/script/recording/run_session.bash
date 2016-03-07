#!/bin/bash

USER_ID="$1"
MODE="$2"
RECORD_FLAG="$3"

#Makes directories for session. 
mkdir -p results/$USER_ID/denotations
mkdir -p results/$USER_ID/phrases
mkdir -p results/$USER_ID/recordings
mkdir -p results/$USER_ID/semantic_forms

#Runs recording program with given arguments.
python record.py $USER_ID $MODE $RECORD_FLAG
