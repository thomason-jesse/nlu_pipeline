#!/usr/bin/python

"""
The functions in this script
are used to preprocess the 
corpora into a desired format.

General preprocessing usage: ./preprocess.py general [corpus_folder] [preprocessed_folder]

    Parameters: 
        corpus_folder       - The ABSOLUTE path to the corpus folder to 
                              be processed. 

        preprocessed_folder - The folder to contain the preprocessed
                              corpus files. This need not be an 
                              absolute path. 

Corpus length filtering usage: ./preprocess.py filter_len [original_corpus_folder] [filtered_corpus_folder] [filtering_length]
    
    Parameters:
        original_corpus_folder - The original corpus folder to filter. Need 
                                 not be an absolute path. 

        filtered_corpus_folder - The folder in which to put the filtered corpus. 

        filtering_length       - The cutoff length to use for the filtering. 

To split a corpus into folds: ./preprocess.py split_into_folds [original_corpus_folder] [fold_corpus_folder] [num_folds]

    Parameters:
        original_corpus_folder - The corpus folder to split into folds. 

        fold_corpus_folder     - The folder in which to create the new corpus
                                 with num_folds folds. 

        num_folds              - The desired number of folds. 

To create a training file for a parser: ./preprocess.py create_parser_training_file [corpus_folder] [parser_training_file_path]
    
    Parameters: 
        corpus_folder             - The folder containing all the corpus files
                                    that will be used to create a parser
                                    training file. 

        parser_training_file_folder - The folder where the training file is to be put.  

author: Rodolfo Corona, rcorona@utexas.edu
"""

__author__ = 'rcorona'

import sys
import os
import glob
import shutil

###########################################################################
################ GENERAL PREPROCESSING FUNCTIONS ##########################
###########################################################################

"""
Preprocesses phrase to get rid
of things such as abbreviations 
that appeared in the corpus so 
that certain word pronunciations 
would be unambiguous during data
collection. 
"""
def preprocess_phrase(phrase):
    #Abbreviations to be expanded. 
    abbreviations = {'P.I.': 'pi', 'T.A.': 'ta', 'B': 'b'}

    preprocessed_phrase = ''

    #Processes phrase into new phrase. 
    for word in phrase:
        if word in abbreviations:
            preprocessed_phrase += abbreviations[word]
        else:
            preprocessed_phrase += word

    #Removes comas, since they are not needed by any system. 
    return preprocessed_phrase.replace(',', '')

"""
This method fixes semantic form errors
that were present in the raw corpora
data. 
"""
def preprocess_semantic_form(semantic_form):
    if semantic_form.startswith('walk(the'):
        semantic_form += '))'

    #Adds CCG category to semantic form. 
    #TODO Current corpus only uses category M, so change if necessary. 
    return 'M : ' + semantic_form

"""
Some semantic forms in the raw corpora were
invalid because of duplicates in the ontology. 
This method ensures a given semantic form is
valid (i.e. does not contain a word which
is a duplicate of an ontology element). 
"""
def semantic_form_valid(semantic_form):
    #Words which were erroneously duplicated in the ontology. 
    duplicates = ["book", "box", "cellphone", "chips",
                  "coffee", "container", "diary",
                  "notebook", "water", "laptop"]

    #Strips category from semantic form. 
    raw_semantic_form = semantic_form.split(': ')[1]

    #Looks for dupliate in the semantic form. 
    for word in duplicates:
        if raw_semantic_form.startswith('bring(' + word):
            return False

    return True

"""
Preprocesses a folder from one of the raw corpora. 

Parameters:
    raw_folder_name - The name of the user folder within the 
                      corpus to preprocess. This MUST
                      be an absolute path. 

    preprocessed_folder_name - The name of the preprocessed corpus
                               folder (e.g. headset, mic, or built_in). 
                               A file text will be created in this folder
                               for each user. This need not be an
                               absolute path. 
                               
"""
def preprocess_folder_general(raw_folder_name, preprocessed_folder_name):
    #Extracts user from raw folder name. 
    user = raw_folder_name.split('/')[-1]

    #Creates preprocessed file for user. 
    preprocessed_file = open(preprocessed_folder_name + '/' + user + '.usr', 'w')

    #Creates a preprocessed file based on the phrase file names. 
    for file_name in os.listdir(raw_folder_name + '/phrases'):
        #Extracts phrase number from file name.  
        number = file_name.split('_')[2]

        #Data point for the phrase. 
        data = ''

        #Gets phrase and writes it. 
        with open(raw_folder_name + '/phrases/' + file_name, 'r') as phrase_file:
            data += preprocess_phrase(phrase_file.readline().strip()) + ';'

        #Gets phrase's semantic form, preprocesses it, and writes it. 
        semantic_form_name = raw_folder_name + '/semantic_forms/' + user + '_semantic_' + str(number)

        with open(semantic_form_name, 'r') as semantic_form_file:
            semantic_form = preprocess_semantic_form(semantic_form_file.readline().strip())
            data += semantic_form + ';'

        #Gets phrase's denotation and writes it.
        denotation_name = raw_folder_name + '/denotations/' + user + '_denotation_' + str(number)

        with open(denotation_name, 'r') as denotation_file:
            data += denotation_file.readline().strip() + ';'

        #Writes corresponding raw recording file path to keep track of. 
        recording_name = raw_folder_name + '/recordings/' + user + '_recording_' + str(number) + '.raw'
        data += recording_name + '\n'

        #Writes the phrase's data if it's semantic form is valid. 
        if semantic_form_valid(semantic_form):
            preprocessed_file.write(data)

    #Closes the preprocessed file.
    preprocessed_file.close()

"""
Preprocesses a corpus folder. 

Parameters:
    corpus_folder - The ABSOLUTE path for the corpus
                    folder which is to be preprocessed.

    preprocessed_folder_name - The path for the folder which
                               is to contain the preprocessed
                               corpus. 
"""
def preprocess_corpus(corpus_folder, preprocessed_folder_name):
    #Used to keep track of progress. 
    folder_count = 1

    #Preprocesses each folder. 
    for user_name in os.listdir(corpus_folder):
        preprocess_folder_general(corpus_folder + '/' + user_name, preprocessed_folder_name)

        #Updates progress. 
        sys.stdout.write('\rPreprocessed ' + str(folder_count) + ' folders...')
        sys.stdout.flush()
        folder_count += 1

    #Puts cursor on next line before terminating. 
    print ''

"""
Computes the number of tokens
present in a phrase based
on how the parser would tokenize them. 
"""
def get_tokenized_phrase_len(phrase):
    length = 0
    
    #Counts individual tokens. (Tokenizes based on appostrophe) 
    for word in phrase.split():
        length += len(word.split("'"))

    return length
        

"""
Creates a filtered corpus from a given one based
on a tokenized phrase length filter. In other
words, if a phrase has more tokens than the given
length (motivated by the parser's parsing speed), 
then it is not included in the filtered corpus.

Parameters:
    corpus_folder - The corpus to filter. 

    filtered_corpus_folder - The folder in which
                             to place the filtered
                             corpus. 

    length - The filtering length to use. 
"""
def filter_corpus_len(corpus_folder, filtered_corpus_folder, length):
    for file_name in os.listdir(corpus_folder):
        #Only processes valid corpus file (i.e. a .usr file). 
        if file_name.endswith('.usr'):
            print 'filtering ' + file_name

            #Opens original for reading and filtered file for writing. 
            original_file = open(corpus_folder + '/' + file_name, 'r')
            filtered_file = open(filtered_corpus_folder + '/' + file_name, 'w')

            #Filters each phrase (i.e. line). 
            for line in original_file:
                #Gets phrase data from original file. 
                phrase, semantic_form, denotation, recording = line.split(';')

                #Writes data if below threshold. 
                if get_tokenized_phrase_len(phrase) < length:
                    filtered_file.write(line)

            #Closes both files. 
            original_file.close()
            filtered_file.close() 

"""
Makes a directory if it doesn't already 
exist. 
"""
def make_dir(directory):
    if not os.path.isdir(directory):
        os.mkdir(directory)

"""
Creates a directory structure
to store experiment train and
test sets in the given 
directory. 
"""
def make_exp_dir_structure(exp_dir):
    #Makes directory itself if it doesn't exist. 
    make_dir(exp_dir)

    #Makes general corpus folder as well as its train and test subfolders. 
    make_dir(exp_dir + '/general')
    make_dir(exp_dir + '/general/train')
    make_dir(exp_dir + '/general/test')

    #Folder to store parser training file. 
    make_dir(exp_dir + '/parser_training')

    #Folder to store models in (e.g parser and language models).
    make_dir(exp_dir + '/models')

    #Folder to store log files in for experiments or training runs.
    make_dir(exp_dir + '/logs')

"""
Takes a corpus folder and creates a
new corpus from it by splitting it
into folds. This new corpus
may be used for cross-fold
validation experiments. 
"""
def split_into_folds(corpus_folder, fold_corpus_folder, num_folds):
    #Gets list of corpus files in directory. 
    corpus_files = glob.glob(corpus_folder + '/*.usr')

    #Ensures corpus may indeed be split into equally sized folds. 
    if len(corpus_files) % num_folds > 0:
        print 'Corpus with ' + str(len(corpus_files)) + ' files will not split into ' + str(num_folds) + ' equal folds!'
        sys.exit()

    #Makes a directory for each fold within the new corpus folder. 
    for i in range(num_folds):
        make_exp_dir_structure(fold_corpus_folder + '/' + str(i))

    #Copies an equal number of files into each fold's train and test sets.
    for i in range(num_folds):
        path = fold_corpus_folder + '/' + str(i)
        counter = 0

        #Puts each corpus file in fold. 
        for corpus_file in corpus_files:
            #True for one out of num_folds files. 
            if (counter % num_folds) == i:
                #Adds to test set. 
                shutil.copy(corpus_file, path + '/general/test/')
            #True (num_folds - 1) out of num_folds files. 
            else:
                #Adds to training set. 
                shutil.copy(corpus_file, path + '/general/train/')

            counter += 1
                
###############################################################################
################## PARSER TRAINING CORPUS RELATED FUNCTIONS ###################
###############################################################################

"""
Creates a training file for a parser
in the desired folder
using all the corpus files found in the 
given corpus_folder. 
"""
def create_parser_training_file(corpus_folder, parser_training_file_folder): 
    parser_training_file = open(parser_training_file_folder + '/parser_train.txt', 'w')

    #Consolidates all corpus data into one file for training. 
    for file_name in os.listdir(corpus_folder):
        if file_name.endswith('.usr'):
            corpus_file = open(corpus_folder + '/' + file_name, 'r')

            #Adds a each training pair in necessary format for parser training. 
            for line in corpus_file:
                phrase, semantic_form, _, _ = line.split(';')

                parser_training_file.write(phrase + '\n' + semantic_form + '\n\n')

    #Closes the training file. 
    parser_training_file.close()

###############################################################################

"""
Simply prints all the usage options for the script. 
"""
def print_usage():
    print 'Usage: ./preprocess.py general [corpus_folder] [preprocessed_folder]'
    print 'Usage: ./preprocess.py filter_len [original_corpus_folder] [filtered_corpus_folder] [filtering_length]'
    print 'Usage: ./preprocess.py split_into_folds [original_corpus_folder] [fold_corpus_folder] [num_folds]'
    print 'Usage: ./preprocess.py create_parser_training_file [corpus_folder] [parser_training_file_folder]'

if __name__ == '__main__':
    if not len(sys.argv) >= 4:
        print_usage()

    #General preprocessing. 
    elif sys.argv[1] == 'general':
        if not len(sys.argv) == 4:
            print 'Usage: ./preprocess.py general [corpus_folder] [preprocessed_folder]' 
        else:
            preprocess_corpus(sys.argv[2], sys.argv[3])
    
    #Corpus length filtering. 
    elif sys.argv[1] == 'filter_len':
        if not len(sys.argv) == 5:
            print 'Usage: ./preprocess.py filter_len [original_corpus_folder] [filtered_corpus_folder] [filtering_length]'
        else:
            filter_corpus_len(sys.argv[2], sys.argv[3], int(sys.argv[4]))

    #Split corpus into equal folds. 
    elif sys.argv[1] == 'split_into_folds':
        if not len(sys.argv) == 5:
            print 'Usage: ./preprocess.py split_into_folds [original_corpus_folder] [fold_corpus_folder] [num_folds]'
        else:
            split_into_folds(sys.argv[2], sys.argv[3], int(sys.argv[4]))
    
    #Create a training file for the parser. 
    elif sys.argv[1] == 'create_parser_training_file':
        if not len(sys.argv) == 4:
            print 'Usage: ./preprocess.py create_parser_training_file [corpus_folder] [parser_training_file_path]'
        else:
            create_parser_training_file(sys.argv[2], sys.argv[3])

    else:
        print_usage()
