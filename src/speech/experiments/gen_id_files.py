#This script generates the id files needed for
#acoustic model adaptation. 

import os
import sys

for k_fold in os.listdir('k-folds'):
    for fold in os.listdir('k-folds/' + k_fold):
        path = 'k-folds/' + k_fold + '/' + fold + '/recordings'

        #Will generate ids for both train and test files. 
        train_file = open(path + '/train/ids.fileids', 'w')
        test_file = open(path + '/test/ids.fileids', 'w')

        #Transcript contains file ids which may be extracted. 
        train_transcript = open(path + '/train/transcript.txt', 'r')
        test_transcript = open(path + '/test/transcript.txt', 'r')

        for line in train_transcript:
            file_id = line.split('(')[1].strip().strip(')')
            train_file.write(file_id + '\n')

        for line in test_transcript:
            file_id = line.split('(')[1].strip().strip(')')
            test_file.write(file_id + '\n')

        train_file.close()
        test_file.close()
        train_transcript.close()
        test_transcript.close()
