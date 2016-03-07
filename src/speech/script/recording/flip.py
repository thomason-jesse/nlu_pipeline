lexFile = open('lex.txt', 'r')
newLex = open('lexF.txt', 'w')

toFlip = {"<I>": 0, "<NO>": 1, "<A>": 2}

def flip():
    line = lexFile.readline()

    while not line.strip() == "</C>":
        backList = line.strip().split(':')
        forwardList = [backList[1].strip(), backList[0].strip()]

        newLine = " : ".join(forwardList) + '\n'

        newLex.write(newLine)

        line = lexFile.readline()

    newLex.write(line)

line = lexFile.readline()

while not line == "":
    if line.strip() in toFlip:
        newLex.write(line)
        flip()
    else:
        newLex.write(line)

    line = lexFile.readline()
