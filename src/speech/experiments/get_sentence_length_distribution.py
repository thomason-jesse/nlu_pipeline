import sys

file = open(sys.argv[1], 'r')

lengths = {}

for line in file:
    if len(line.strip()) > 0 and not line.startswith("M"):
        length = len(line.split())

        if length in lengths:
            lengths[length] += 1
        else:
            lengths[length] =1

total = 0
below_11 = 0

for length in lengths:
    total += lengths[length]

    if length < 11:
        below_11 += lengths[length]


print str(lengths)
print "TOTAL: " + str(total)
print "BELOW 11: " + str(below_11)
