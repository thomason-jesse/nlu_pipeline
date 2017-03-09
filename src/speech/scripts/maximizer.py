#!/usr/bin/python

f1_file = open('tuning/f1.txt', 'r')

averages = {}

for line in f1_file:
    weight, scores = line.strip().split(':')

    scores = [float(score) for score in scores.split(';')]

    average = float(sum(scores)) / float(len(scores))
    
    averages[weight] = average

maximum = float('-inf')
max_weight = None

for weight in averages: 
    if averages[weight] > maximum:
        maximum = averages[weight]
        max_weight = weight

print [max_weight, maximum]
