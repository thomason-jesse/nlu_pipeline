#!/usr/bin/env python
__author__ = 'jesse'

import rospy
import sys
import Ontology
import Lexicon
import KBGrounder
import Parser
import StaticDialogPolicy
import ActionSender
from TemplateBasedGenerator import TemplateBasedGenerator
from StaticDialogAgent import StaticDialogAgent

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

print "instantiating KBGrounder"
grounder = KBGrounder.KBGrounder(ont)

print "instantiating Parser"
parser = Parser.Parser(ont, lex, grounder)
d = parser.read_in_paired_utterance_semantics(sys.argv[3])
converged = parser.train_learner_on_semantic_forms(d, 10, reranker_beam=10)
if not converged:
    raise AssertionError("Training failed to converge to correct values.")

print "instantiating DialogAgent"
u_in = InputFromKeyboard()
u_out = OutputToStdout()
static_policy = StaticDialogPolicy.StaticDialogPolicy()
A = StaticDialogAgent(parser, grounder, static_policy, u_in, u_out)

response_generator = TemplateBasedGenerator()

while True:
    u_out.say("How can I help?")
    s = raw_input()
    if s == 'stop':
        break
    a = A.initiate_dialog_to_get_action(s)
    print response_generator.get_action_sentence(a)
