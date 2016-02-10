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
parser = CKYParser.CKYParser(ont, lex)

print "reading in data and beginning test"
d = parser.read_in_paired_utterance_semantics(sys.argv[3])
converged = parser.train_learner_on_semantic_forms(d, 30)
if not converged:
    raise AssertionError("Training failed to converge to correct values.")
