__author__ = 'jesse'

import os
import sys
import time

tests = [('aish',
          'python tests/aish/main.py tests/aish/ont.txt tests/aish/lex.txt tests/aish/samples.txt'),
         ('templated_examples',
          'python tests/templated_examples/main.py tests/templated_examples/ont.txt tests/templated_examples/lex.txt tests/templated_examples/10.txt'),
         ('cky_parser_basics',
          'python tests/cky_parser_basics/main.py tests/cky_parser_basics/ont.txt \
          tests/cky_parser_basics/lex.txt tests/cky_parser_basics/samples.txt \
          tests/cky_parser_basics/test.txt'),
         ('grounding_recall',
         'rosrun nlu_pipeline src/tests/grounding_recall/main.py tests/grounding_recall/ont.txt \
          tests/grounding_recall/lex.txt tests/grounding_recall/samples.txt'),
         ('action_grounding',
         'rosrun nlu_pipeline src/tests/action_grounding/main.py tests/action_grounding/ont.txt \
          tests/action_grounding/lex.txt tests/action_grounding/utterance_action_pairs.txt'),
         ('navigation',
          'python tests/navigation/main.py tests/navigation/ont.txt \
          tests/navigation/lex.txt tests/navigation/samples.txt'),
         ('merge_op',
          'python tests/merge_op/main.py tests/merge_op/parser_1000 tests/merge_op/samples.txt')]

# read command line arguments
tests_to_run = [t[0] for t in tests]
tests_to_output = []
tests_to_debug = []
for i in range(1, len(sys.argv)):
    if sys.argv[i] == '-run':
        tests_to_run = sys.argv[i + 1].split(',')
        i += 1
    if sys.argv[i] == '-both':
        tests_to_debug = sys.argv[i + 1].split(',')
        tests_to_output = sys.argv[i + 1].split(',')
        i += 1
    if sys.argv[i] == '-stderr':
        tests_to_debug = sys.argv[i + 1].split(',')
        i += 1
    if sys.argv[i] == '-stdout':
        tests_to_output = sys.argv[i + 1].split(',')
        i += 1
    if sys.argv[i] == '-h':
        sys.exit("python run_tests.py [-run/-stderr/-stdout " + ",".join([t[0] for t in tests]) + "]")

# read expected completion times
tests_info = {}
try:
    f = open('run_tests_data.txt', 'r')
    for line in f:
        test, curr_avg, total_runs = line.strip().split(',')
        tests_info[test] = [curr_avg, total_runs]
    f.close()
except IOError:
    pass

completion_times = {}
for tn in tests_to_run:
    t = None
    for ts in tests:
        if ts[0] == tn:
            t = ts
            break
    print "RUNNING TEST:\t" + t[0]
    if t[0] in tests_info:
        print "... expected completion time: "+str(tests_info[t[0]][0])+" seconds"
    c = t[1]
    if t[0] not in tests_to_output and 'all' not in tests_to_output:
        c += " > /dev/null"
    if t[0] not in tests_to_debug and 'all' not in tests_to_debug:
        c += " 2> /dev/null"
    bt = time.time()
    r = os.system(c)
    et = time.time()
    if r != 0:
        print "... FAILED: " + str(r)
    else:
        print "... PASSED"
        completion_times[t[0]] = et - bt

# write new expected completion times
written = []
f = open('run_tests_data.txt', 'w')
for t in tests_info:
    if t in completion_times:
        f.write(",".join([t,
                         str((float(tests_info[t][0])*int(tests_info[t][1]) + completion_times[t])
                             / (int(tests_info[t][1])+1)),
                         str(int(tests_info[t][1])+1)])+"\n")
        written.append(t)
    else:
        f.write(",".join([t, tests_info[t][0], tests_info[t][1]])+"\n")
for t in tests:
    if t[0] not in written:
        if t[0] in completion_times:
            f.write(",".join([t[0],
                              str(completion_times[t[0]]),
                              "1"])+"\n")
f.close()
