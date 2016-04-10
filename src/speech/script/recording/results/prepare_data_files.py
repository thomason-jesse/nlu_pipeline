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

    return return_phrase.strip() + "\n"

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

for user in os.listdir('headset'):
    path = 'headset/' + user + '/'
        
    #Opens files to write consolidated data to.
    phrase_file = open(path + user + '_phrases.txt', 'w')
    denotation_file = open(path + user + '_denotations.txt', 'w')
    semantic_forms_file = open(path + user + '_semantic_forms.txt', 'w')

    num_files = len(os.listdir(path + 'recordings/'))

    for i in range(1, num_files + 1):
        phrase = open(path + 'phrases/' + user + '_phrase_' + str(i), 'r')
        phrase_file.write(process_phrase(phrase.read(), 1))
        phrase.close()

        denotation = open(path + 'denotations/' + user + '_denotation_' + str(i), 'r')
        denotation_file.write(denotation.read() + '\n')
        denotation.close()

        semantic_form = open(path + 'semantic_forms/' + user + '_semantic_' + str(i), 'r')
        semantic_forms_file.write(semantic_form.read() + '\n')
        semantic_form.close()

    phrase_file.close()
    denotation_file.close()
    semantic_forms_file.close()

    #Prepares parser training file for user. 
    training_file = open(path + user + '_train.txt', 'w')
    phrase_file = open(path + user + '_phrases.txt', 'r')
    denotation_file = open(path + user + '_denotations.txt', 'r')
    semantic_forms_file = open(path + user + '_semantic_forms.txt', 'r')

    for i in range(1, num_files + 1):
       training_file.write(process_phrase(phrase_file.readline(), 2))
       training_file.write(semantic_forms_file.readline())
       training_file.write(denotation_file.readline() + '\n')

    training_file.close()
    phrase_file.close()
    denotation_file.close()
    semantic_forms_file.close()
