#This script splits a file id file in two. 

import sys
import os

path = sys.argv[1]
k = int(sys.argv[2].split('_')[0])
num_per_set = (32 / k) / 2

read_file = open(path + 'ids.fileids', 'r')
ids1 = open(path + 'ids1.fileids', 'w')
ids2 = open(path + 'ids2.fileids', 'w')

num_so_far = 0
prev = ""
write_file = ids1

for line in read_file:
    write_file.write(line)

    if len(line.strip()) == 0 and not prev.strip() == "":
        num_so_far += 1

        if num_so_far == num_per_set:
            write_file = ids2

    prev = line

read_file.close()
ids1.close()
ids2.close()

