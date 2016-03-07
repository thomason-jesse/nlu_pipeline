corrections = open("corrections.txt", "r")
logs = open("noNamesLogs.txt", "r")
corrected = open("correctedLogs.txt", "w")

correctList = {}

for line in corrections:
    splitLine = line.split(":")
    error = splitLine[0]
    replacement = " ".join(splitLine[1].split())

    correctList[error] = replacement

for line in logs:
    sentence = []

    for word in line.split(" "):
        if word in correctList:
            sentence.append(correctList[word])
        else:
            sentence.append(word)

    corrected.write(" ".join(sentence))

corrections.close()
logs.close()
corrected.close()
