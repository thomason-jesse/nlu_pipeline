wordsFile = open('words.txt', 'r')

tagDescriptions = {'<FN>': "First Name",
                   '<LN>': "Last Name",
                   '<I>': "Item",
                   '<P>': "Place"}

tags = {}

for line in wordsFile:
    wordAndTag = line.split(':')

    if not wordAndTag[1].rstrip() in tags:
        tags[wordAndTag[1].rstrip()] = []

    #Adds word to list of words with tag. 
    tags[wordAndTag[1].rstrip()].append(wordAndTag[0])

for tag in tags:
    print str(tag) + ": " + str(tags[tag])

lexFile = open('lex.txt', 'w')

for tag in tagDescriptions:
    lexFile.write(tag + '\n' + '___\n')

    for word in tags[tag]:
        lexFile.write(word + '\n')

wordsFile.close()
lexFile.close()
