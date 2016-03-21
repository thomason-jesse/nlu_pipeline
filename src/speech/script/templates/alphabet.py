lexFile = open("lex.txt", "r")
alphabeticalFile = open("lexA.txt", "w")

line = lexFile.readline()

def alphabetize(lexFile):
    line = lexFile.readline()
    l = []

    while not line.rstrip() == "</B>":
        l.append(line)
        line = lexFile.readline()

    sortedList = sorted(l)

    for entry in sortedList:
        alphabeticalFile.write(entry)


while not line == "":
    if line.rstrip() == "<B>":
        alphabetize(lexFile)
    else:
        alphabeticalFile.write(line)

    line = lexFile.readline()

lexFile.close()
alphabeticalFile.close()

