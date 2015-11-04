import sys

template = open("template.txt", "r")
namedLogs = open("concreteLogs.txt", "w")

firstNames = ["ray", "peter", "dana"]
firstNamesPos = ["ray's", "peter's", "dana's"]
lastNames = ["mooney", "stone", "ballard"]
lastNamesPos = ["mooney's", "stone's", "ballard's"]

numNames = len(firstNames)

lists = {"<FIRST_NAME>": firstNames,
         "<FIRST_NAME_S>": firstNamesPos,
         "<LAST_NAME>": lastNames,
         "<LAST_NAME_S>": lastNamesPos}

def writeNames(line):
    for num in range(0, numNames):
        sentence = []

        for word in line.split(" "):
            if word in lists:
                sentence.append(lists[word][num])
            else:
                sentence.append(word)

        namedLogs.write(" ".join(sentence))

for line in template:
    replace = False

    for word in line.split(" "):
        if word in lists:
            replace = True
            break

    if replace:
        writeNames(line)
    else:
        namedLogs.write(line)

template.close()
namedLogs.close()
