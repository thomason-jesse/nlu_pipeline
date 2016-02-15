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
import DialogAgent
import numpy, re

from PomdpDialogAgent import PomdpDialogAgent
from Utils import *

class InputFromKeyboard:
    def __init__(self):
        pass

    def get(self):
        text = raw_input()
        text = text.lower()
        regex = re.compile('[\?\.,\;\:]')
        text = regex.sub('', text)
        return text 


class OutputToStdout:
    def __init__(self):
        pass

    def say(self, s):
        print "SYSTEM: "+s

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

#print '\n\n', predicate_holds('room', 'l3_432', grounder), '\n\n'

print "instantiating Generator"
generator = Generator.Generator(ont, lex, learner, parser, beam_width=100)
#print "testing Generator:"
#while True:
#    s = raw_input()
#    if s == 'stop':
#        break
#    form = lex.read_semantic_form_from_str(s, None, None, [])
#    token_responses = generator.reverse_parse_semantic_form(form, k=3, n=1)
#    print "token responses: "+str(token_responses)

print "instantiating DialogAgent"
u_in = InputFromKeyboard()
u_out = OutputToStdout()
#static_policy = StaticDialogPolicy.StaticDialogPolicy()
#A = DialogAgent.DialogAgent(parser, grounder, static_policy, u_in, u_out)
A = PomdpDialogAgent(parser, grounder, u_in, u_out)

print "reading in data and training parser from actions"
D = A.read_in_utterance_action_pairs(sys.argv[3])
converged = A.train_parser_from_utterance_action_pairs(D, epochs=10, parse_beam=30)
print "theta: "+str(parser.learner.theta)
save_model(parser, 'parser')
#print 'Parser ontology : ', parser.ontology.preds

# Testing typechecking
#idx = grounder.ontology.preds.index('ray')
#person_idx = grounder.ontology.preds.index('person')
#child = SemanticNode(None, 9, 15, False, idx)
#parent = SemanticNode(None, 9, 15, False, person_idx, children=[child])
#child.parent = parent  
#print "Grounding ", parser.print_parse(parent)
#g = grounder.groundSemanticNode(parent, [], [], [])
#answers = grounder.grounding_to_answer_set(g)
#print 'answers = ', answers
#print '--------------------------------'
#print predicate_holds('person', 'ray', grounder)
#print predicate_holds('person', 'l3_512', grounder)
#print predicate_holds('room', 'ray', grounder)
#print predicate_holds('room', 'l3_512', grounder)
#sys.exit(1)

print 'Lexicon - ', parser.lexicon.surface_forms, '\n\n'

#print '\n\nCorrect system running\n\n'

while True:
    A.first_turn = True
    success = A.run_dialog()
    if not success :
        u_out.say("I'm sorry I could not help you. Do you want to try another dialogue? (y/n) : ")
        response = u_in.get()   
        if response.lower() == 'n' or response.lower() == 'no' :
            break
    else :
        u_out.say("Do you want to try another dialogue? (y/n) : ")
        response = u_in.get()   
        if response.lower() == 'n' or response.lower() == 'no' :
            break 

#print "testing Generator:"
#while True:
#    s = raw_input()
#    if s == 'stop':
#        break
#    form = lex.read_semantic_form_from_str(s, None, None, [])
#    token_responses = generator.reverse_parse_semantic_form(form, k=3, n=3)
#    print "token responses: "+str(token_responses)
