#!/bin/bash

USER_ID="$1"

#Makes directories for session. 
mkdir -p results/built_in/$USER_ID/denotations
mkdir -p results/built_in/$USER_ID/phrases
mkdir -p results/built_in/$USER_ID/recordings
mkdir -p results/built_in/$USER_ID/semantic_forms

#Runs recording program with given arguments.
python record44100Hz.py $USER_ID results/built_in/$USER_ID y y
