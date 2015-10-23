import linecache

corrections = open("corrections.txt", "w")
misspellings = open("misspellings.txt", "r")
logs = "noNamesLogs.txt"

for line in misspellings:
    splitLine = line.split(":")
    wordLineNum = int(splitLine[0])
    word = "".join(splitLine[1].split())

    wordLine = linecache.getline("words.txt", wordLineNum)
    phraseLineNum = int(wordLine.split(":")[0].rstrip())
    phrase = linecache.getline(logs, phraseLineNum)

    print phrase.rstrip()
    print word
    correction = raw_input("Correction: ")

    output = word + ": " + correction + " \n"
    
    print output
    corrections.write(output)

corrections.close()
