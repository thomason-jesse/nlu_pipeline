#!/usr/bin/env python
__author__ = 'aishwarya'

import sys
import Ontology
import Lexicon
import FeatureExtractor
import LinearLearner
import KBGrounder
import Parser
import Generator
import numpy, re

from PomdpTrainer import PomdpTrainer
from Utils import *

# Fixing the random seed for debugging
numpy.random.seed(4)

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
#parser = load_model('parser')
grounder.parser = parser
grounder.ontology = parser.ontology

print "instantiating Generator"
generator = Generator.Generator(ont, lex, learner, parser, beam_width=100)

print "instantiating Trainer"
A = PomdpTrainer(parser, grounder)

print "reading in data and training parser from actions"
D = A.read_in_utterance_action_pairs(sys.argv[3])
converged = A.train_parser_from_utterance_action_pairs(D, epochs=10, parse_beam=30)
print "theta: "+str(parser.learner.theta)
save_model(parser, 'parser')

success_dir = '/u/aish/Documents/Research/rlg/logs_only/second/valid'
fail_dir = '/u/aish/Documents/Research/rlg/logs_only/second/invalid'
A.train_from_old_logs(success_dir, fail_dir)
