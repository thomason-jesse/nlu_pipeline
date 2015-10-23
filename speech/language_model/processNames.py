preLogs = open("usrLogsWordNums.txt", "r")
postLogs = open("noNamesLogs.txt", "w")

first = ["mallory",
         "walter",
         "peggy",
         "alice",
         "robert",
         "carol",
         "dave",
         "evelyn"]

last = ["morgan", 
        "ward",
        "parker",
        "ashcraft",
        "brown",
        "clark",
        "daniels",
        "ellis"]

for line in preLogs:
    words = []

    for word in line.split(' '):
        if word in first:
            words.append("<FIRST_NAME>")
        elif word in last:
            words.append("<LAST_NAME>")
        else:
            words.append(word)

    postLogs.write(" ".join(words))

preLogs.close()
postLogs.close()
