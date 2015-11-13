#!/usr/bin/env python
__author__ = 'jesse'

import rospy
import sys
import Ontology
import Lexicon
import FeatureExtractor
import LinearLearner
import KBGrounder
import Parser
import Generator
import DialogAgent
import StaticDialogPolicy
import ActionSender


class InputFromKeyboard:
    def __init__(self):
        pass

    def get(self):
        return raw_input()


class OutputToStdout:
    def __init__(self):
        pass

    def say(self, s):
        print "SYSTEM: "+s

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
parser = Parser.Parser(ont, lex, learner, grounder, beam_width=10, safety=True)

print "instantiating Generator"
generator = Generator.Generator(ont, lex, learner, parser, beam_width=sys.maxint, safety=True)

u_in = InputFromKeyboard()
u_out = OutputToStdout()

print "testing Generator:"
while True:
    s = raw_input()
    if s == 'stop':
        break
    _, form = lex.read_syn_sem(s)
    token_responses = generator.reverse_parse_semantic_form(form, n=1, c=1)
    print "token responses: "+str(token_responses)
generator.flush_seen_nodes()

while True:
    u_out.say("How can I help?")
    s = raw_input()
    if s == 'stop':
        break
    top_n = parser.parse_expression(s, k=10, n=3, allow_UNK_E=False)
    print "num parses: "+str(len(top_n))
    for r, ps, ph, sc in top_n:
        parser.print_parse(r, True)

print "reading in training data"
D = parser.read_in_paired_utterance_semantics(sys.argv[3])

if len(sys.argv) > 4 and sys.argv[4] == "both":
    print "training parser and generator jointly from semantics"
    parser.train_learner_on_semantic_forms(D, k=10, generator=generator)
else:
    print "training parser from semantics"
    parser.train_learner_on_semantic_forms(D, k=10)

print "theta: "+str(parser.learner.theta)

while True:
    u_out.say("How can I help?")
    s = raw_input()
    if s == 'stop':
        break
    top_n = parser.parse_expression(s, k=10, n=3, allow_UNK_E=False)
    print "num parses: "+str(len(top_n))
    for r, ps, ph, sc in top_n:
        parser.print_parse(r, True)

print "testing Generator:"
while True:
    s = raw_input()
    if s == 'stop':
        break
    _, form = lex.read_syn_sem(s)
    token_responses = generator.reverse_parse_semantic_form(form, n=1)
    print "token responses: "+str(token_responses)

