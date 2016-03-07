"""
This file runs through the dialogue data
and generates a list of each unique word
that occurs inside it. 
"""

dataFile = open('data.txt', 'r')
wordsFile = open('words.txt', 'w')

unique = {}

for line in dataFile:
    words = line.split()

    for word in words:
        unique[word] = "Here"

for word in unique:
    wordsFile.write(word + '\n')

wordsFile.close()
dataFile.close()
