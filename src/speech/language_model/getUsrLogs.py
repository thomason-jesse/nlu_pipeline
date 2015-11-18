logs = open("logs.txt", "r")
usrLogs = open("usrLogsRaw.txt", "w")


for line in logs:
    parts = line.split("\t", 1)

    if parts[0] == "USER":
        usrLogs.write(parts[1])

logs.close()
usrLogs.close()
