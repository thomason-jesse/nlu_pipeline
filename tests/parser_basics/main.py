__author__ = 'jesse'

import sys

sys.path.append('.')  # necessary to import local libraries
import Ontology, Lexicon, FeatureExtractor, LinearLearner, Parser

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

print "instantiating Parser"
parser = Parser.Parser(ont, lex, learner, grounder=None, beam_width=100)

print "reading in data and beginning test"
D = parser.read_in_paired_utterance_semantics(sys.argv[3])
for [x, y] in D:
    print "parsing " + str(x)
    k_best = parser.parse_tokens(x)
    correct_found = False
    for ans in k_best:
        if y.equal_ignoring_syntax(ans[0]):
            correct_found = True
            break
    if not correct_found:
        print "mismatch on tokens " + str(x)
        print "correct form: "
        print parser.print_parse(y)
        print "predicted forms:"
        for ans in k_best:
            print ans[2]
            print parser.print_parse(ans[0])
            print parser.print_semantic_parse_result(ans[1])
        raise AssertionError("Correct parse was not generated for " + str(x))
