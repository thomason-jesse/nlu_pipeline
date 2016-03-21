"""
This program processes all the tags
and generates a file with unique
utterance templates. 
"""

taggedFile = open('taggedList.txt', 'r')
tags = {}

#Generates list of tag words. 
for line in taggedFile:
    wordAndTag = line.split(':')
    tags[wordAndTag[0]] = wordAndTag[1].rstrip()

taggedFile.close()


dataFile = open('data.txt', 'r')
templatesFile = open('dirtyTemplates.txt', 'w')

#Keeps unique templates. 
templates = {}

#Replaces occurrences of tag words with their generic tags. 
for line in dataFile:
    words = line.split()
    newLine = ""

    for word in words:
        if word in tags:
            newLine += str(tags[word])
        else:
            newLine += word

        newLine += ' '

    newLine += '\n'

    templates[newLine] = "Template"

for template in templates:
    templatesFile.write(template)

print "Num dirty templates: " + str(len(templates))

dataFile.close()
templatesFile.close()


