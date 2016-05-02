#!/usr/bin/python

#This script creates transcripts for each fileid file. 

import os
import sys

path = sys.argv[1]

transcript = open(path + "transcript.txt", "r")
transcript_1 = open(path + "transcript1.txt", 'w')
transcript_2 = open(path + "transcript2.txt", 'w')

ids1 = open(path + "ids1.fileids", 'r')
ids2 = open(path + 'ids2.fileids', 'r')

for line in ids1:
    transcript_1.write(transcript.readline())

transcript_1.close()

for line in ids2:
    transcript_2.write(transcript.readline())

transcript_2.close()

ids1.close()
ids2.close()
transcript.close()
