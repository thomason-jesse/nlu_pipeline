import sys

logs = open(sys.argv[1], "r")
wordFile = open("words.txt", "w")

words = {}
firstTime = {}

num = 1

for line in logs:
    for word in line.split(" "):
        if word in words:
            words[word] += 1
        else:
            words[word] = 1
            firstTime[word] = num
    
    num += 1

for word in words:
    result = word + ": " + str(words[word]) + "\n"
    print result

    wordFile.write(str(firstTime[word]) + " : " + word + " \n")

logs.close()
wordFile.close()

            
