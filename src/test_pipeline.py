#!/usr/bin/env python
__author__ = 'jesse'

import rospy
import sys
import Ontology
import Lexicon
import KBGrounder
import CKYParser
import StaticDialogPolicy
import ActionSender
from TemplateBasedGenerator import TemplateBasedGenerator
from StaticDialogAgent import StaticDialogAgent
from utils import *

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
parser = CKYParser.CKYParser(ont, lex, use_language_model=True)
# Set parser hyperparams to best known values for training
parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
parser.max_new_senses_per_utterance = 3  # max number of new word senses that can be induced on a training example
parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
parser.max_hypothesis_categories_for_unknown_token_beam = 3  # for unknown token, max syntax categories tried
d = parser.read_in_paired_utterance_semantics(sys.argv[3])
converged = parser.train_learner_on_semantic_forms(d, 10, reranker_beam=10)
if not converged:
    raise AssertionError("Training failed to converge to correct values.")
save_model(parser, 'parser')
# parser = load_model('parser')

# Set parser hyperparams to best known values for test time
parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
parser.max_cky_trees_per_token_sequence_beam = 1000  # for tokenization of an utterance, max cky trees considered
parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried

grounder.parser = parser
grounder.ontology = parser.ontology

print "instantiating DialogAgent"
u_in = InputFromKeyboard()
u_out = OutputToStdout()
static_policy = StaticDialogPolicy.StaticDialogPolicy()
A = StaticDialogAgent(parser, grounder, static_policy, u_in, u_out)
A.dialog_objects_logfile = 'src/nlu_pipeline/src/models/trial_log.pkl'

response_generator = TemplateBasedGenerator()

while True:
    u_out.say("How can I help?")
    s = raw_input()
    if s == 'stop':
        break
    a = A.initiate_dialog_to_get_action(s)
    if a is not None :
        print response_generator.get_action_sentence(a)
