import sys

if not len(sys.argv) == 2:
    print "Expected one argument [txt file]"
    sys.exit()

txtFileN = sys.argv[1]

logs = open("logs.txt", "a")
txtFile = open(txtFileN, "r")

for line in txtFile:
    logs.write(line.rstrip("\n") + " \n")

logs.close()
txtFile.close()
