import sys
import os

def process_phrase(phrase, pass_num):
    words = phrase.strip().split()
    
    #Used to concatenate numbers. 
    whole_num = None

    #Processed phrase to return. 
    return_phrase = ""

    for word in words:
        if (not word in numbers) or pass_num == 1:
            #If previous words made up number, then write it to file. 
            if not whole_num == None:
                return_phrase += whole_num + " "
                whole_num = None

            if word in abbreviations:
                return_phrase += abbreviations[word] + " "
            else:
                return_phrase += word + " "
        else:
            #Either begins or continues constructing full number from words. 
            if whole_num == None:
                whole_num = numbers[word]
            else:
                whole_num += numbers[word]
 

    if not whole_num == None:
        return_phrase += whole_num

    #For speech. 
    if pass_num == 3:
        return_phrase = '<s> ' + return_phrase.strip() + ' </s>'

    return return_phrase.strip() + "\n"

def validSemanticForm(semantic_form):
    for word in duplicates:
        if semantic_form.startswith("bring(" + word):
            return False

    return True

#Dictionary to post-process numbers. 
numbers = {"zero": "0", "one": "1", "two": "2", "three": "3",
           "four": "4", "five": "5", "six": "6",
           "seven": "7", "eight": "8", "nine": "9",
           "thirty-four": "34", "B": "b", 
           "fourteen": "14", "eighteen": "18",
           "twenty": "20", "thirty-two": "32",
           "thirty-five": "35", "ten": "10",
           "twelve": "12", "sixteen": "16"}

abbreviations = {"P.I.": "pi", "T.A.": "ta"}

duplicates = ["book", "box", "cellphone", "chips",
              "coffee", "container", "diary", 
              "notebook", "water"]

for user in os.listdir('headset'):
    path = 'headset/' + user + '/'
        
    #Opens files to write consolidated data to.
    phrase_file = open(path + user + '_phrases.txt', 'w')
    denotation_file = open(path + user + '_denotations.txt', 'w')
    semantic_forms_file = open(path + user + '_semantic_forms.txt', 'w')
    recordings_file = open(path + user + '_recording_files.txt', 'w')

    num_files = len(os.listdir(path + 'phrases/'))

    #Only gets 100 data points per user.
    i = 1
    num_written = 0
    while num_written < 95:
        phrase_buff = open(path + 'phrases/' + user + '_phrase_' + str(i), 'r')
        phrase = process_phrase(phrase_buff.read(), 1)
        phrase_buff.close()


        denotation_buff = open(path + 'denotations/' + user + '_denotation_' + str(i), 'r')
        denotation = denotation_buff.read()
        denotation_buff.close()


        semantic_form_buff = open(path + 'semantic_forms/' + user + '_semantic_' + str(i), 'r')
        semantic_form = semantic_form_buff.read()
        semantic_form_buff.close()

        #Fixes format bug. 
        if semantic_form.startswith("walk(the"):
            #Parentheses added to fix bug. They were miscounted during corpus generation :(
            semantic_form += "))"
        
        #Writes data if semantic form is valid (i.e. because of duplicate items in ontology error.)
        if validSemanticForm(semantic_form):
            num_written += 1
            recordings_file.write(path + 'recordings/' + user + '_recording_' + str(i) + '.raw\n') 
            phrase_file.write(phrase)
            denotation_file.write(denotation + '\n')
            semantic_forms_file.write('M : ' + semantic_form + '\n')

        i += 1

    phrase_file.close()
    denotation_file.close()
    semantic_forms_file.close()
    recordings_file.close()

    #Prepares parser training file for user. 
    parser_training_file = open(path + user + '_train_parser.txt', 'w')
    lm_training_file = open(path + user + '_train_lm.txt', 'w')

    phrase_file = open(path + user + '_phrases.txt', 'r')
    denotation_file = open(path + user + '_denotations.txt', 'r')
    semantic_forms_file = open(path + user + '_semantic_forms.txt', 'r')

    for i in range(1, num_files + 1):
        phrase = phrase_file.readline()    
        
        lm_training_file.write(process_phrase(phrase, 3))

        parser_training_file.write(process_phrase(phrase, 2))
        parser_training_file.write(semantic_forms_file.readline() + '\n')
        #parser_training_file.write(denotation_file.readline() + '\n')

    lm_training_file.close()
    parser_training_file.close()
    phrase_file.close()
    denotation_file.close()
    semantic_forms_file.close()
