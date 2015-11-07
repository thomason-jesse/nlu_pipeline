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
import pygame
import threading
from std_msgs.msg import String

class InputFromKeyboard:
    def __init__(self):
        pass

    def get(self):
        return raw_input()

class InputFromSpeechNode:
    def __init__(self):
        #Subscribes to speech node. 
        rospy.Subscriber('speech', String, self.callback)
        self.utterance = ""
        self.waiting = True

    def callback(self, data):
        self.utterance = data.data.split(";")[0]

        self.waiting = False

    def get(self):
        #Waits for utterance message to arrive. 
        while self.waiting:
            pass

        print self.utterance

        self.waiting = True

        return self.utterance


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
parser = Parser.Parser(ont, lex, learner, grounder, beam_width=10, safety=True)

print "instantiating Generator"
generator = Generator.Generator(ont, lex, learner, parser, beam_width=sys.maxint, safety=True)
print "testing Generator:"
while True:
    s = raw_input()
    if s == 'stop':
        break
    _, form = lex.read_syn_sem(s)
    token_responses = generator.reverse_parse_semantic_form(form, n=1, c=1)
    print "token responses: "+str(token_responses)
generator.flush_seen_nodes()

print "instantiating DialogAgent"
u_in = InputFromSpeechNode()
u_out = OutputToStdout()
static_policy = StaticDialogPolicy.StaticDialogPolicy()
A = DialogAgent.DialogAgent(parser, generator, grounder, static_policy, u_in, u_out)

print "instantiating ActionSender"
action_sender = ActionSender.ActionSender(lex, generator, u_out)

while True:
    u_out.say("How can I help?")
    s = u_in.get()
    if s == 'stop':
        break
    a = A.initiate_dialog_to_get_action(s)
    print "ACTION: "+str(a)
    r = action_sender.take_action(a)
    print "RESULT: "+str(r)

print "reading in training data"
D = A.read_in_utterance_action_pairs(sys.argv[3])

if len(sys.argv) > 4 and sys.argv[4] == "both":
    print "training parser and generator jointly from actions"
    converged = A.jointly_train_parser_and_generator_from_utterance_action_pairs(
        D, epochs=10, parse_beam=30, generator_beam=10)
else:
    print "training parser from actions"
    converged = A.train_parser_from_utterance_action_pairs(
        D, epochs=10, parse_beam=30)

print "theta: "+str(parser.learner.theta)

while True:
    u_out.say("How can I help?")
    s = u_in.get()
    if s == 'stop':
        break
    a = A.initiate_dialog_to_get_action(s)
    print "ACTION: "+str(a)
    r = action_sender.take_action(a)
    print "RESULT: "+str(r)

print "testing Generator:"
while True:
    s = u_in.get()
    if s == 'stop':
        break
    _, form = lex.read_syn_sem(s)

    token_responses = generator.reverse_parse_semantic_form(form, n=1)
    print "token responses: "+str(token_responses)
