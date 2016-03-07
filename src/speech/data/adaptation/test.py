import sys

sentences = open('sentences.txt', 'r')
transcripts = open('transcription.txt', 'w')
ids = open('adaptation.fileids', 'w')

lines = sentences.readlines()
num = 0

for line in lines:
    transcripts.write('<s> ' + line.rstrip() + ' </s> (file' + str(num) + ')\n')
    ids.write('file' + str(num) + '\n')
    num += 1
