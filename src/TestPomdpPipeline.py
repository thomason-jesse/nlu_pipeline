#!/usr/bin/env python
__author__ = 'aishwarya'

import sys
import Ontology
import Lexicon
import KBGrounder
import CKYParser
import numpy, re

from Knowledge import Knowledge
from PomdpKtdqPolicy import PomdpKtdqPolicy
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

print "instantiating KBGrounder"
grounder = KBGrounder.KBGrounder(ont)

#print "instantiating Parser"
#parser = CKYParser.CKYParser(ont, lex)
#d = parser.read_in_paired_utterance_semantics(sys.argv[3])
#converged = parser.train_learner_on_semantic_forms(d, 10, reranker_beam=10)
#if not converged:
    #raise AssertionError("Training failed to converge to correct values.")
#save_model(parser, 'parser')
parser = load_model('parser')
parser.max_new_senses_per_utterance = 5

print "instantiating DialogAgent"
u_in = InputFromKeyboard()
u_out = OutputToStdout()

knowledge = Knowledge()
policy = PomdpKtdqPolicy(knowledge, False)
A = PomdpDialogAgent(parser, grounder, policy, u_in, u_out)
A.dialog_objects_logfile = 'src/nlu_pipeline/src/models/trial_log.pkl'

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

