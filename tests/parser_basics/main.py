__author__ = 'jesse'

import sys
import copy

sys.path.append('.')  # necessary to import local libraries
import Ontology, Lexicon, FeatureExtractor, LinearLearner, Parser

print "reading in Ontology"
ont = Ontology.Ontology(sys.argv[1])
commutative_idxs = [ont.preds.index('and'), ont.preds.index('or')]
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
        if y.equal_allowing_commutativity(ans[0], commutative_idxs, ontology=ont):
            correct_found = True
            break
    if not correct_found:
        print "mismatch on tokens " + str(x)
        print "correct form: "
        print parser.print_parse(y, show_non_lambda_types=True)
        print "predicted forms:"
        for ans in k_best:
            B = copy.deepcopy(ans[0])
            B.commutative_raise_node(commutative_idxs)
            print parser.print_parse(ans[0], show_non_lambda_types=True)
            print parser.print_semantic_parse_result(ans[1])
        raise AssertionError("Correct parse was not generated for " + str(x))
