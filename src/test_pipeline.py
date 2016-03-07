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
<<<<<<< HEAD
from TemplateBasedGenerator import TemplateBasedGenerator
from StaticDialogAgent import StaticDialogAgent
from DialogAgent import DialogAgent
from utils import *
=======
import pygame
import os
import threading
import pickle
from std_msgs.msg import String
>>>>>>> fec3ba2bcc5a8fcff5845323a760282e0ec1a761

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
        
        #Creates list of nbest hypotheses returned by Sphinx. 
        self.utterance = data.data.split(";")

        self.waiting = False

    def get(self):
        #Waits for utterance message to arrive. 
        while self.waiting:
            pass

        self.waiting = True

        return self.utterance


class OutputToStdout:
    def __init__(self):
        pass

    def say(self, s):
        print "SYSTEM: "+s

<<<<<<< HEAD
# Fixing the random seed for debugging
numpy.random.seed(10)
=======
class OutputWithSpeech:
    def __init__(self):
        pass
 
    def say(self, s):
        
        #Prints and synthesizes dialogue.
        print s

        os.system("espeak '" + s + "'")

#Path for argument files. 
path = './src/nlu_pipeline/src/speech/data/parser/'
>>>>>>> fec3ba2bcc5a8fcff5845323a760282e0ec1a761

print "calling ROSpy init"
rospy.init_node('test_NLU_pipeline')

print "reading in Ontology"
ont = Ontology.Ontology(path + 'ont.txt')
print "predicates: " + str(ont.preds)
print "types: " + str(ont.types)
print "entries: " + str(ont.entries)

print "reading in Lexicon"
lex = Lexicon.Lexicon(ont, path + 'lex.txt')
print "surface forms: " + str(lex.surface_forms)
print "categories: " + str(lex.categories)
print "semantic forms: " + str(lex.semantic_forms)
print "entries: " + str(lex.entries)

print "instantiating KBGrounder"
grounder = KBGrounder.KBGrounder(ont)

<<<<<<< HEAD
#print "instantiating Parser"
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
#parser = load_model('parser')

# Set parser hyperparams to best known values for test time
parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
parser.max_cky_trees_per_token_sequence_beam = 1000  # for tokenization of an utterance, max cky trees considered
parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried

grounder.parser = parser
grounder.ontology = parser.ontology
=======
print "instantiating Parser from pickle file"
pickledParser = open(path + 'parser.pickle', 'r')
parser = pickle.load(pickledParser)
#parser = Parser.Parser(ont, lex, learner, grounder, beam_width=10, safety=True)

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
>>>>>>> fec3ba2bcc5a8fcff5845323a760282e0ec1a761

print "instantiating DialogAgent"
u_in = InputFromSpeechNode()
u_out = OutputWithSpeech()
static_policy = StaticDialogPolicy.StaticDialogPolicy()
A = StaticDialogAgent(parser, grounder, static_policy, u_in, u_out)
#A = DialogAgent(parser, grounder, static_policy, u_in, u_out)
A.dialog_objects_logfile = 'src/nlu_pipeline/src/models/trial_log.pkl'

response_generator = TemplateBasedGenerator()

while True:
    u_out.say("How can I help?")
    s = u_in.get()
    if s == 'stop':
        break
    a = A.initiate_dialog_to_get_action(s)
<<<<<<< HEAD
    if a is not None :
        print response_generator.get_action_sentence(a)
=======
    print "ACTION: "+str(a)
    r = action_sender.take_action(a)
    print "RESULT: "+str(r)

print "reading in training data"
D = A.read_in_utterance_action_pairs(path + 'utterance_action_pairs.txt')

#if len(sys.argv) > 4 and sys.argv[4] == "both":
if False:
    print "training parser and generator jointly from actions"
    converged = A.jointly_train_parser_and_generator_from_utterance_action_pairs(
        D, epochs=10, parse_beam=30, generator_beam=10)
else:
    print "training parser from actions"
    converged = A.train_parser_from_utterance_action_pairs(
        D, epochs=10, parse_beam=30)

print "theta: "+str(parser.learner.theta)

#Pickles trained parser. 
parserFile = open(path + 'parser.pickle', 'w')
pickle.dump(parser, parserFile)
print "Pickled parser"

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
>>>>>>> fec3ba2bcc5a8fcff5845323a760282e0ec1a761
