#!/usr/bin/env python
__author__ = 'jesse'

import sys

sys.path.append('.')  # necessary to import local libraries
import Ontology
import Lexicon
import CKYParser
import KBGrounder

print "reading in Ontology"
ont = Ontology.Ontology(sys.argv[1])
print "predicates: " + str(ont.preds)
print "types: " + str(ont.types)
print "entries: " + str(ont.entries)

print "reading in Lexicon"
lex = Lexicon.Lexicon(ont, sys.argv[2])
print "surface forms: " + str(lex.surface_forms)
print "categories: " + str(lex.categories)
print "semantic forms: " + str(lex.semantic_forms)
print "entries: " + str(lex.entries)

print "instantiating KBGrounder"
grounder = KBGrounder.KBGrounder(ont)

print "instantiating Parser"
parser = CKYParser.CKYParser(ont, lex)

f = open(sys.argv[3])
f_lines = f.readlines()
f.close()
i = 0
while i < len(f_lines):
    if len(f_lines[i].strip()) == 0:
        i += 1
        continue
    u = f_lines[i].strip()
    d = [entry.strip() for entry in f_lines[i + 1].strip().split(";")]
    print "running parser on '" + u + "' expected denotations " + str(d)
    parse_generator = parser.most_likely_cky_parse(u)
    g = []
    l = 10  # limit how many parse possibilities we consider before testing against known denotations
    for p, s, _ in parse_generator:
        if p is None:
            break
        print str(s) + ":\nparse: " + parser.print_parse(p.node, show_category=True)
        print "running grounder on parse..."
        new_g = grounder.groundSemanticNode(p.node, [], [], [])
        print new_g
        g.extend([str(ng) for ng in new_g])
        l -= 1
        if l == 0:
            break
    for denotation_instance in d:
        if denotation_instance not in g:
            raise AssertionError(
                "Denotation '" + str(denotation_instance) + "' not successfully grounded for '" + u + "'")
    i += 2
