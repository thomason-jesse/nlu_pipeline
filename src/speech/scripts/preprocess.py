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

        filtering_length       - The cutoff length to use for the filtering, inclusive. 
                                 In other words: <= filtering_length

To split a corpus into folds: ./preprocess.py split_into_folds [original_corpus_folder] [fold_corpus_folder] [num_folds]

    Parameters:
        original_corpus_folder - The corpus folder to split into folds. 

        fold_corpus_folder     - The folder in which to create the new corpus
                                 with num_folds folds. 

        num_folds              - The desired number of folds. 

To create a training file for a parser: ./preprocess.py create_parser_training_file [corpus_folder] [parser_training_file_path] [Optional: filter_length]
    
    Parameters: 
        corpus_folder             - The folder containing all the corpus files
                                    that will be used to create a parser
                                    training file. 

        parser_training_file_folder - The folder where the training file is to be put.  

        filter_length               - An optional parameter. If given, is the maximum 
                                      number of tokens that will be accepted when 
                                      pulling phrases to put in the training file. 

To create a training file for a language model: ./preprocess.py create_lm_training_file [corpus_folder] [lm_training_file_folder]
    
    Parameters: 
        corpus_folder               - The folder containing all the corpus files
                                      that will be used to create a parser
                                      training file. 

        lm_training_file_folder     - The folder where the training file is to be put. 

To create transcription files for a folder of .usr files: ./preprocess.py create_transcript_files [corpus_folder] [transcript_files_folder]

    Parameters:
        corpus_folder               - The top level folder which contains .usr files
                                      to create transcription files for. 

        transcript_files_folder     - The folder in which the transcription files
                                      are to be placed. 

To create ASR training files from a .usr file folder: ./preprocess.py create_asr_test_files [usr_files_folder] [test_files_folder] [num_test_files]

    Parameters:
        usr_files_folder            - The folder containing the .usr files to use to create the test files. 

        test_files_folder           - The folder in which to place the generated test files. 

        num_test_files              - The number of test files to generate. 

author: Rodolfo Corona, rcorona@utexas.edu
"""

__author__ = 'rcorona'

import sys
import os
import glob
import shutil
import post_process
from experiments import tokenize_for_parser

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
    for word in phrase.split():
        if word in abbreviations:
            preprocessed_phrase += abbreviations[word] + ' '
        else:
            preprocessed_phrase += word + ' ' 

    #Removes comas, since they are not needed by any system. 
    return preprocessed_phrase.replace(',', '').strip()

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
        data += os.path.abspath(recording_name) + '\n'

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
                if get_tokenized_phrase_len(phrase) <= length:
                    filtered_file.write(line)

            #Closes both files. 
            original_file.close()
            filtered_file.close() 

"""
Makes a directory if it doesn't already 
exist. 
"""
def make_dir(directory):
    #Converts to absolute path. 
    directory = os.path.abspath(directory)

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
    make_dir(exp_dir + '/corpus')
    make_dir(exp_dir + '/corpus/train')
    make_dir(exp_dir + '/corpus/test')

    #Folder to store parser training file. 
    make_dir(exp_dir + '/parser_training')

    #Folder to store models in (e.g parser and language models).
    make_dir(exp_dir + '/models')
    make_dir(exp_dir + '/models/adapted_ac_model')
    make_dir(exp_dir + '/models/re_trained_baseline_ac_model')
    make_dir(exp_dir + '/models/re_trained_cky_ac_model')

    #Folder to store log files in for experiments or training runs.
    make_dir(exp_dir + '/logs')
    make_dir(exp_dir + '/logs/training/')
    make_dir(exp_dir + '/logs/experiments/')
    make_dir(exp_dir + '/logs/evaluations/')

    #Folder to store ASR related training files. 
    make_dir(exp_dir + '/asr_training')

    #Folder to store experiment related files. 
    make_dir(exp_dir + '/experiments')

    #Contains ASR test files. 
    make_dir(exp_dir + '/experiments/asr')
    
    #Testing files for ASR
    make_dir(exp_dir + '/experiments/asr/test_files')

    #Result files from running ASR on test files. 
    make_dir(exp_dir + '/experiments/asr/result_files')

    #Keeps files resulting from evaluating experiment results. 
    make_dir(exp_dir + '/evaluations')

    #Adds directories for all experiment types.
    make_dir(exp_dir + '/evaluations/wer')
    make_dir(exp_dir + '/evaluations/top_1')
    make_dir(exp_dir + '/evaluations/top_5')
    make_dir(exp_dir + '/evaluations/sem_full')
    make_dir(exp_dir + '/evaluations/sem_partial')

"""
Takes a preprocessed corpus folder and creates a
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

    #Makes fold corpus folder if necessary. 
    make_dir(fold_corpus_folder)

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
                shutil.copy(corpus_file, path + '/corpus/test/')
            #True (num_folds - 1) out of num_folds files. 
            else:
                #Adds to training set. 
                shutil.copy(corpus_file, path + '/corpus/train/')

            counter += 1
                
###############################################################################
################## PARSER TRAINING CORPUS RELATED FUNCTIONS ###################
###############################################################################

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

def get_tokenized_phrase(phrase):

    #Currently only processing needed to tokenize. 
    return phrase.replace("'", " '")

"""
Creates a training file for a parser
in the desired folder
using all the corpus files found in the 
given corpus_folder. 
"""
def create_parser_training_file(corpus_folder, parser_training_file_folder, filter_len=None): 
    parser_training_file = open(parser_training_file_folder + '/parser_train.txt', 'w')

    #Gets phrase length limit.
    if filter_len == None:
        length_lim = float('inf')
    else:
        length_lim = int(filter_len)

    #Consolidates all corpus data into one file for training. 
    for file_name in os.listdir(corpus_folder):
        if file_name.endswith('.usr'):
            corpus_file = open(corpus_folder + '/' + file_name, 'r')

            #Adds a each training pair in necessary format for parser training. 
            for line in corpus_file:
                phrase, semantic_form, _, _ = line.split(';')

                #Adds phrase if it is under the token length limit. 
                if get_tokenized_phrase_len(phrase) <= length_lim:
                    parser_training_file.write(get_tokenized_phrase(phrase) + '\n' + semantic_form + '\n\n')

    #Closes the training file. 
    parser_training_file.close()

"""
Adds type constraints to a .usr file for use
by parser. 
"""
def add_type_constraints_to_usr_file(file_name):
    old_file = open(file_name, 'r')
    tmp_file = open('temp.txt', 'w')

    for line in old_file:
        line_elements = line.split(';')    

        #Gets semantic form and replaces it. 
        new_semantic_form = post_process.add_type_constraints(line.split(';')[1].split('M : ')[1].strip()) 
        line_elements[1] = 'M : ' + new_semantic_form 

        #Writes new semantic form to new file. 
        tmp_file.write(';'.join(line_elements))

    tmp_file.close()

    #Overwrites old file with new one, now containing type constraints. 
    os.rename('temp.txt', file_name)

###############################################################################

###############################################################################
#################### ASR RELATED FUNCTIONS ###########################
###############################################################################

"""
Prepares a desired number of test files that will be used with the ASR
by splitting all the usr files into the desired number of sets. 
In other words, if you want two or n test sets, use this function. 
"""
def create_asr_test_files(usr_files_folder, test_files_folder, num_test_files, basename, extension):
    #TODO add back in to functionality if needed. 
    filter_len = float('inf')

    #String input cast to int.
    num_test_files = int(num_test_files)

    #Creates list of usr files to use for asr test files. 
    usr_files = []

    for file_name in os.listdir(usr_files_folder):
        if file_name.endswith('.usr'):
            usr_files.append(usr_files_folder + '/' + file_name)

    #Ensures usr files are evenly divisible by desired number of test files. 
    if len(usr_files) % num_test_files > 0:
        print "Number of usr files does not evenly divide desired number of test files!"
        sys.exit()

    #Splits usr files into equal-sized bins for test files. 
    test_files_lists = {}

    for i in range(num_test_files):
        test_files_lists[i] = []

    for i in range(len(usr_files)):
        test_files_lists[i % num_test_files].append(usr_files[i])

    #Writes test files. 
    for i in range(num_test_files):
        test_file = open(test_files_folder + '/' + basename + str(i) + '.' + extension, 'w')

        for usr_file_name in test_files_lists[i]:
            usr_file = open(usr_file_name, 'r')

            #Adds phrase to 
            for line in usr_file:
                tok_phrase_len = len(tokenize_for_parser(line.split(';')[0]).split())

                if tok_phrase_len <= filter_len:  
                    test_file.write(line)

        test_file.close()
                

def create_lm_training_file(corpus_folder, lm_training_file_folder):
    lm_training_file = open(lm_training_file_folder + '/lm_train.txt', 'w')

    #Consolidates all corpus data into one file for training. 
    for file_name in os.listdir(corpus_folder):
        if file_name.endswith('.usr'):
            corpus_file = open(corpus_folder + '/' + file_name, 'r')

            #Adds each phrase to file with start and end of sentence tokens. 
            for line in corpus_file:
                phrase, _, _, _ = line.split(';')

                lm_training_file.write('<s> ' + phrase + ' </s>' + '\n')

    #Closes the training file. 
    lm_training_file.close()

"""
Extracts a transcript from an experiment result file. 
Used for re-training an acoustic model. 
"""
def extract_transcript_from_results_file(file_name, ids_file_name, transcript_file_name, test_file_name):
    #Open files for reading / writing. 
    results_file = open(file_name, 'r')
    test_file = open(test_file_name, 'r')
    ids_file = open(ids_file_name, 'w')
    transcript_file = open(transcript_file_name, 'w')

    #Used for retrieving a result. 
    retrieve_flag = 0
    
    for line in results_file:
        # '#' delimits the first line for a phrase result, next line will be top hypothesis. 
        if line.startswith('#'):
            retrieve_flag = 1

        #If retrieve flag is set, then fetch hypothesis. 
        elif retrieve_flag == 1:
            #Get recording file path from test file. 
            phrase, _, _, recording_path = test_file.readline().strip().split(';')
            recording_path = recording_path.split('headset/')[1].split('.')[0]

            #Get hypothesis string. 
            hyp_string = line.split(';')[0]

            #Write formatted data to transcripts and id files.
            transcript_file.write('<s> ' + hyp_string + ' </s> (' + recording_path + ')\n')
            ids_file.write(recording_path + '\n')

            #Set to zero so that next hypotheses aren't fetched. 
            retrieve_flag = 0

    results_file.close()
    ids_file.close()
    transcript_file.close()
    test_file.close()

"""
Extracts necessary transcript information
from usr file and writes it to given
ids and transcript files. 
"""
def extract_transcripts(file_name, ids_file, transcript_file):
    #Opens file and gets user's name. 
    usr_file = open(file_name, 'r')
    usr_name = file_name.split('/')[-1].split('.')[0]
    
    #Creates an id and transcription for each phrase. 
    for line in usr_file:
        phrase, _, _, recording_path = line.split(';')

        #Extracts file id from recording name. 
        file_id = usr_name + '/recordings/' + recording_path.strip().split('/')[-1].split('.')[0] + '\n'

        #Composes phrase transcription for file. 
        transcription = '<s> ' + phrase + ' </s> (' + file_id.strip() + ')\n'

        #Writes them to their respective files.
        ids_file.write(file_id)
        transcript_file.write(transcription)

"""
Create transcription files that will be used
by Sphinx to either adapt an accoustic model
or calculate WER. 
"""
def create_asr_transcripts(corpus_folder, transcript_files_folder):
    name = 'train'

    #Will contain relative paths for recordings according to top level folder. 
    #Used by Sphinx to coordinate accoustic model adaptation and WER computations. 
    ids_file = open(transcript_files_folder + '/' + name + '.fileids', 'w')

    #Will contain file ids along with their transcription. 
    transcript_file = open(transcript_files_folder + '/' + name + '.transcription', 'w')

    #Goes through each user file and extracts necessary information to create files. 
    for file_name in os.listdir(corpus_folder):
        if file_name.endswith('.usr'):
            extract_transcripts(corpus_folder + '/' + file_name, ids_file, transcript_file)

    #Closes up files. 
    ids_file.close()
    transcript_file.close()

##################################################################################

"""
Simply prints all the usage options for the script. 
"""
def print_usage():
    print 'Usage:'
    print 'general preprocessing: ./preprocess.py general [corpus_folder] [preprocessed_folder]'
    print 'Phrase length filtering: ./preprocess.py filter_len [original_corpus_folder] [filtered_corpus_folder] [filtering_length]'
    print 'Fold splitting: ./preprocess.py split_into_folds [original_corpus_folder] [fold_corpus_folder] [num_folds]'
    print 'Parser train file: ./preprocess.py create_parser_training_file [corpus_folder] [parser_training_file_folder] [Optional: filter_length]'
    print 'LM train file:  ./preprocess.py create_lm_training_file [corpus_folder] [lm_training_file_folder]'
    print 'Corpus transcript files: ./preprocess.py create_transcript_files [corpus_folder] [transcript_files_folder]'
    print 'Create ASR testing files: ./preprocess.py create_asr_test_files [usr_files_folder] [test_files_folder] [num_test_files] [file basename] [file extension]'

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
        if len(sys.argv) == 4:
            create_parser_training_file(sys.argv[2], sys.argv[3])
        elif len(sys.argv) == 5:
            create_parser_training_file(sys.argv[2], sys.argv[3], sys.argv[4])
        else:
            print 'Usage: ./preprocess.py create_parser_training_file [corpus_folder] [parser_training_file_path] [Optional: filter_length]'

    #Create a training file for a language model. 
    elif sys.argv[1] == 'create_lm_training_file':
        if len(sys.argv) == 4:
            create_lm_training_file(sys.argv[2], sys.argv[3])
        else:
            print 'LM train file:  ./preprocess.py create_lm_training_file [corpus_folder] [lm_training_file_folder]'

    #Create transcript files for a corpus. 
    elif sys.argv[1] == 'create_transcript_files':
        if len(sys.argv) == 4:
            create_asr_transcripts(sys.argv[2], sys.argv[3])
        else:
            print 'Corpus transcript files: ./preprocess.py create_transcript_files [corpus_folder] [transcript_files_folder]'

    #Create test files for running experiments. 
    elif sys.argv[1] == 'create_asr_test_files':
        if len(sys.argv) == 7:
            create_asr_test_files(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])
        else:
            print_usage()

    else:
        print_usage()
