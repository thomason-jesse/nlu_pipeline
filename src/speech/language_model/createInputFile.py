import sys

logs = open(sys.argv[1], "r")
inputFile = open("data.txt", "w")

for line in logs:
    inputFile.write("<s> " + line.rstrip() + " </s>\n")

logs.close()
inputFile.close()


