__author__ = 'jesse'

import sys

sys.path.append('.')  # necessary to import local libraries
import Ontology
import Lexicon
import CKYParser

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

print "instantiating CKYParser"
parser = CKYParser.CKYParser(ont, lex, use_language_model=True)

print "reading in data and beginning training test"
d = parser.read_in_paired_utterance_semantics(sys.argv[3])
converged = parser.train_learner_on_semantic_forms(d, 10, reranker_beam=10)
if not converged:
    raise AssertionError("Training failed to converge to correct values.")

print "reading in data and beginning evaluation test"
d = parser.read_in_paired_utterance_semantics(sys.argv[4])
for [x, y] in d:
    print "testing on '"+x+"' targeting "+parser.print_parse(y, show_category=True)
    parse_generator = parser.most_likely_cky_parse(x, reranker_beam=10)
    correct = False
    for i in range(0, 10):
        best, score, _ = next(parse_generator)
        language_score = parser.get_language_model_score(best)
        if best is None:
            raise AssertionError("Testing failed on example '"+x+"' for which no more parses remain")
        if y.equal_allowing_commutativity(best.node, parser.commutative_idxs, ontology=ont):
            correct = True
            break
        else:
            print "...generated incorrect parse "+parser.print_parse(best.node, show_category=True) + \
                " with score "+str(score)+", language score "+str(language_score)
    if not correct:
        raise AssertionError("Testing failed on example '" + x + "' while failed to give correct form in beam")
