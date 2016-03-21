import sys

numbers = {'0': "zero",
           '1': "one", 
           '2': "two", 
           '3': "three", 
           '4': "four", 
           '5': "five",
           '6': "six", 
           '7': "seven", 
           '8': "eight", 
           '9': "nine"}

usrLogs = open("usrLogsRaw.txt", "r")
cleanUsrLogs = open("usrLogsWordNums.txt", "w")

def postProcess(words):
    utterance = []

    for word in words:
        number = False

        for char in word:
            if char in numbers:
                number = True
                break

        if number:
            for char in word:
                if char in numbers:
                    utterance.append(numbers[char])
                else:
                    utterance.append(char)
        else:
            utterance.append(word)

    return " ".join(utterance)

for line in usrLogs:
    cleanUsrLogs.write(postProcess(line.split(' ')))

usrLogs.close()
cleanUsrLogs.close()
