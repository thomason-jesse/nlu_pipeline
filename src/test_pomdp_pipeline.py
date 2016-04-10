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
from utils import *

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
#parser = CKYParser.CKYParser(ont, lex, use_language_model=True)
## Set parser hyperparams to best known values for training
#parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
#parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
#parser.max_cky_trees_per_token_sequence_beam = 1000  # for tokenization of an utterance, max cky trees considered
#parser.max_hypothesis_categories_for_unknown_token_beam = 5  # for unknown token, max syntax categories tried
#parser.max_expansions_per_non_terminal = 5 # max number of backpointers stored per nonterminal per cell in CKY chart
#d = parser.read_in_paired_utterance_semantics(sys.argv[3])
#converged = parser.train_learner_on_semantic_forms(d, 10, reranker_beam=10)
#if not converged:
    #raise AssertionError("Training failed to converge to correct values.")
#save_model(parser, 'parser')
parser = load_model('both_parser')

# Set parser hyperparams to best known values for test time
parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried
parser.max_expansions_per_non_terminal = 5 # max number of backpointers stored per nonterminal per cell in CKY chart

grounder.parser = parser
grounder.ontology = parser.ontology

print "instantiating DialogAgent"
u_in = InputFromKeyboard()
u_out = OutputToStdout()

knowledge = Knowledge()
#policy = PomdpKtdqPolicy(knowledge, False)
policy = load_model('both_policy')
policy.untrained = False
policy.training = False
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

