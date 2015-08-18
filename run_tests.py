import os, sys

tests = [('parser_basics',
          'python tests/parser_basics/main.py tests/parser_basics/ont.txt tests/parser_basics/lex.txt tests/parser_basics/samples.txt'),
        ('grounding_recall',
          'rosrun bwi_dialog src/tests/grounding_recall/main.py tests/grounding_recall/ont.txt tests/grounding_recall/lex.txt tests/grounding_recall/samples.txt')]

tests_to_output = []
tests_to_debug = []
for i in range(1, len(sys.argv)):
    if sys.argv[i] == '-stderr':
        tests_to_debug = sys.argv[i + 1].split(',')
        i += 1
    if sys.argv[i] == '-stdout':
        tests_to_output = sys.argv[i + 1].split(',')
        i += 1
    if sys.argv[i] == '-h':
        sys.exit("python run_tests.py [-stderr/-stdout " + ",".join([t[0] for t in tests]) + "]")

for t in tests:
    print "RUNNING TEST:\t" + t[0]
    c = t[1]
    if t[0] not in tests_to_output: c += " > /dev/null"
    if t[0] not in tests_to_debug: c += " 2> /dev/null"
    r = os.system(c)
    if r != 0:
        print "...FAILED: " + str(r)
    else:
        print "...PASSED"
