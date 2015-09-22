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

print "calling ROSpy init"
rospy.init_node('test_NLU_pipeline')

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

print "instantiating Generator"
generator = Generator.Generator(ont, lex, learner, parser, beam_width=100)
print "testing Generator:"
while True:
    s = raw_input()
    if s == 'stop':
        break
    form = lex.read_semantic_form_from_str(s, None, None, [])
    token_responses = generator.reverse_parse_semantic_form(form, k=3, n=1)
    print "token responses: "+str(token_responses)

print "instantiating DialogAgent"
u_in = InputFromKeyboard()
u_out = OutputToStdout()
static_policy = StaticDialogPolicy.StaticDialogPolicy()
A = DialogAgent.DialogAgent(parser, grounder, static_policy, u_in, u_out)

print "instantiating ActionSender"
action_sender = ActionSender.ActionSender(lex, generator, u_out)

while True:
    u_out.say("How can I help?")
    s = raw_input()
    if s == 'stop':
        break
    a = A.initiate_dialog_to_get_action(s)
    print "ACTION: "+str(a)
    r = action_sender.take_action(a)
    print "RESULT: "+str(r)

print "reading in data and training parser from actions"
D = A.read_in_utterance_action_pairs(sys.argv[3])
converged = A.train_parser_from_utterance_action_pairs(D, epochs=10, parse_beam=30)
print "theta: "+str(parser.learner.theta)

while True:
    u_out.say("How can I help?")
    s = raw_input()
    if s == 'stop':
        break
    a = A.initiate_dialog_to_get_action(s)
    print "ACTION: "+str(a)
    r = action_sender.take_action(a)
    print "RESULT: "+str(r)

print "testing Generator:"
while True:
    s = raw_input()
    if s == 'stop':
        break
    form = lex.read_semantic_form_from_str(s, None, None, [])
    token_responses = generator.reverse_parse_semantic_form(form, k=3, n=3)
    print "token responses: "+str(token_responses)
