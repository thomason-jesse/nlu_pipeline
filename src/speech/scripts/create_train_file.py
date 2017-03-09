#!/usr/bin/python

import sys

if __name__ == '__main__':
    length = int(sys.argv[1])

    train_file = open('parser_train.txt', 'r')
    new_train_file = open('parser_train' + str(length) + '.txt', 'w')

    lines = [line for line in train_file]

    train_file.close()

    index = 0

    while index < len(lines): 
        if len(lines[index].strip()) == 0: 
            pass
        elif lines[index].startswith('M : ') and len(phrase.replace("'s", " 's").split()) == length:
            semantic_form = lines[index].strip()
            new_train_file.write(phrase + '\n' + semantic_form + '\n\n')
        else: 
            phrase = lines[index].strip()
   
        index += 1
            
    new_train_file.close()
