import os.path

def process(utterance):
    utterance = utterance.rstrip()

    #TODO add instructions to clean up utterance. (i.e. names, contractions, etc.)

    return utterance

corpus = open('corpus.txt', 'w')
path = 'parsing_results/'
num = 0
name = str(num) + path + '-parsing.txt'


#Formats and adds every parsing result to corpus. 
while os.path.isfile(name):
    utteranceFile = open(name, 'r')
    utterance = process(utteranceFile.read())

    corpus.write('<s> ' + utterance + ' </s>\n')

    num += 1
    name = str(num) + path + '-parsing.txt'

    

