#!/usr/bin/env python

import sys

sys.path.append('.')  # necessary to import local libraries
import Ontology
import Lexicon
import FeatureExtractor
import LinearLearner
import KBGrounder
import Parser
import DialogAgent

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
parser = Parser.Parser(ont, lex, learner, grounder, beam_width=10)

print "instantiating DialogAgent"
A = DialogAgent.DialogAgent(parser, grounder, None, None, None)

print "reading in data and training parser from actions"
D = A.read_in_utterance_action_pairs(sys.argv[3])
converged = A.train_parser_from_utterance_action_pairs(D, epochs=10, parse_beam=30)
print "theta: "+str(parser.learner.theta)

if not converged:
    raise AssertionError("Training from utterance/action pairs failed to converge")
