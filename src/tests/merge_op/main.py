__author__ = 'jesse'

import sys
sys.path.append('.')  # necessary to import local libraries
from utils import *

print "loading parser from provided file..."
parser = load_model(sys.argv[1].split('/')[-1], path='/'.join(sys.argv[1].split('/')[:-1]))
if parser is None:
    raise IOError("Could not load model file '" + str(sys.argv[1]) + ".pkl'")
print "... done"

# test is passed if parser doesn't crash on any samples
print "reading and running test examples..."
f = open(sys.argv[2], 'r')
print "\n===\n"
for line in f:
    parse_getter = parser.most_likely_cky_parse(line)
    p = next(parse_getter)
    print line
    print parser.print_parse(p[0].node)+'\n\n===\n'
print "... done"
