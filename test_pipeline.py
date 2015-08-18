#!/usr/bin/env python

import sys
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
parser = Parser.Parser(ont, lex, learner, grounder, beam_width=100)

print "reading in data and training parser"
D = parser.read_in_paired_utterance_denotation(sys.argv[3])
parser.train_learner_on_denotations(D, epochs=10)
print parser.learner.theta

print "instantiating DialogAgent"
A = DialogAgent.DialogAgent(parser, grounder)

while True:
    print "\nsentence to be parsed:"
    s = raw_input()
    if s == 'stop': break
    print "RESPONSE: "+str(A.respond_to_utterance(s))
