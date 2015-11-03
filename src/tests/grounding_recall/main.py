#!/usr/bin/env python
__author__ = 'jesse'

import sys

sys.path.append('.')  # necessary to import local libraries
import Ontology, Lexicon, FeatureExtractor, LinearLearner, Parser, KBGrounder

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

print "instantiating Feature Extractor"
f_extractor = FeatureExtractor.FeatureExtractor(ont, lex)

print "instantiating Linear Learner"
learner = LinearLearner.LinearLearner(ont, lex, f_extractor)

print "instantiating KBGrounder"
grounder = KBGrounder.KBGrounder(ont)

print "instantiating Parser"
parser = Parser.Parser(ont, lex, learner, grounder, beam_width=100)

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
    k_best = parser.parse_expression(u, n=100)
    print "k_best = " + str(k_best)
    g = []
    for [p, spr, trace, s] in k_best:
        print str(s) + ":\nparse: " + parser.print_parse(p)
        print "Parse Tree:\n" + parser.print_semantic_parse_result(spr) + "\n"
        print "running grounder on parse..."
        new_g = grounder.groundSemanticNode(p, [], [], [])
        print new_g
        g.extend([str(ng) for ng in new_g])
    for denotation_instance in d:
        if denotation_instance not in g:
            raise AssertionError(
                "Denotation '" + str(denotation_instance) + "' not successfully grounded for '" + u + "'")
    i += 2
