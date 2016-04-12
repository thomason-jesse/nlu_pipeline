#This script takes the fold data files from each
#fold in corpus directory and makes the training and
#test sets to be used in each fold during experiments.

import sys
import os
import shutil

#Appends file1 to file2
def append(file1, file2):
    for line in file1:
        file2.write(line)

#Generates files for all k's. 
for k_fold_folder in os.listdir('corpus'):
    k = int(k_fold_folder.split('_')[0])

    #Reads in data files for each fold. 
    file_list = {}
    path = 'corpus/' + k_fold_folder

    for i in range(1, k + 1):
        file_list[i] = {}

        file_list[i]['parser'] = path + '/' + str(i) + '/' + str(i) + '_train_parser.txt'
        file_list[i]['lm'] = path + '/' + str(i) + '/' + str(i) + '_train_lm.txt'
        file_list[i]['recordings'] = path + '/' + str(i) + '/' + str(i) + '_recording_files.txt'
            
    #Creates training and test files for each fold.
    for i in range(1, k + 1):
        path = 'k-folds/' + str(k) + '_fold/' + str(i) 
        parser_train_file = open(path + '/parser_train.txt', 'w')
        lm_train_file = open(path + '/lm_train.txt', 'w')
        recording_files = open(path + '/recordings_train.txt', 'w')

        for j in range(1, k + 1):
            #Appends data files from all folds except test fold. 
            if not j == i:
                append(open(file_list[j]['lm'], 'r'), lm_train_file)
                append(open(file_list[j]['parser'], 'r'), parser_train_file)
                append(open(file_list[j]['recordings'], 'r'), recording_files)

        #Creates test files by copying folds data files.  
        shutil.copyfile(file_list[i]['lm'], path + '/lm_test.txt')
        shutil.copyfile(file_list[i]['parser'], path + '/parser_test.txt')
        shutil.copyfile(file_list[i]['recordings'], path + '/recordings_test.txt')
