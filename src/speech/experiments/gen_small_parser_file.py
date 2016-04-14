import sys
import os

parser_path = sys.argv[1]

original = open(parser_path + "/parser_train.txt", "r")
new = open(parser_path + "/parser_train_small.txt", "w")

i = 0

while i < 50:
    line = original.readline()
    new.write(line)

    if len(line.strip()) == 0:
        i += 1

original.close()
new.close()
