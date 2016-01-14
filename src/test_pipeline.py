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
from TemplateBasedGenerator import TemplateBasedGenerator

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
parser = Parser.Parser(ont, lex, learner, grounder, beam_width=10, safety=True)

print "instantiating Generator"
generator = Generator.Generator(ont, lex, learner, parser, beam_width=sys.maxint, safety=True)
#print "testing Generator:"
#while True:
    #s = raw_input()
    #if s == 'stop':
        #break
    #_, form = lex.read_syn_sem(s)
    #token_responses = generator.reverse_parse_semantic_form(form, n=1, c=1)
    #print "token responses: "+str(token_responses)
#generator.flush_seen_nodes()

print "instantiating DialogAgent"
u_in = InputFromKeyboard()
u_out = OutputToStdout()
static_policy = StaticDialogPolicy.StaticDialogPolicy()
A = DialogAgent.DialogAgent(parser, generator, grounder, static_policy, u_in, u_out)

print "instantiating ActionSender"
action_sender = ActionSender.ActionSender(lex, generator, u_out)

#while True:
    #u_out.say("How can I help?")
    #s = raw_input()
    #if s == 'stop':
        #break
    #a = A.initiate_dialog_to_get_action(s)
    #print "ACTION: "+str(a)
    #r = action_sender.take_action(a)
    #print "RESULT: "+str(r)

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

response_generator = TemplateBasedGenerator()

print '\n\n', 'Trying to add to lexicon'
# Training the parser
#tokens = A.parser.tokenize('burger') 
#semantic_form = A.parser.lexicon.read_semantic_form_from_str('hamburger', None, None, [], allow_expanding_ont=False)
##print 'Ontology entry: ', A.parser.lexicon.ontology.preds[semantic_form.idx]
#examples = [[tokens, semantic_form]]
#A.parser.generator_genlex = True
#A.parser.train_learner_on_semantic_forms(examples, generator=generator)

# Directly adding to the lexicon
#example = 'burger :- N : hamburger'
#lines = [example]
#print 'surface forms = ', A.parser.lexicon.surface_forms
#A.parser.lexicon.expand_lex_from_strs(
            #lines, A.parser.lexicon.surface_forms, A.parser.lexicon.semantic_forms, 
            #A.parser.lexicon.entries, A.parser.lexicon.pred_to_surface, 
            #allow_expanding_ont=False)
#print 'surface forms = ', A.parser.lexicon.surface_forms

#if 'burger' in A.parser.lexicon.surface_forms :
    #example_surface_idx = A.parser.lexicon.surface_forms.index('burger')
    #print 'example_surface_idx = ', example_surface_idx
    #print 'type(A.parser.lexicon.entries) = ', type(A.parser.lexicon.entries)
    #if len(A.parser.lexicon.entries) > example_surface_idx:
        #semantic_form_indices = A.parser.lexicon.entries[example_surface_idx]
        #semantic_forms = [A.parser.lexicon.semantic_forms[idx] for idx in semantic_form_indices]
        #ont_entries = [A.parser.lexicon.ontology.preds[semantic_form.idx] for semantic_form in semantic_forms]
        #print 'Matching ontology entry = ', ont_entries
    #else :
        #print 'No entry for surface form'
#else :
    #print 'Surface form not in lexicon'

#print 'Lexicon: ', [A.parser.lexicon.ontology.preds[form.idx] for form in A.parser.lexicon.semantic_forms if form.idx is not None]
#print 'Ontology: ', A.parser.lexicon.ontology.preds

while True:
    u_out.say("How can I help?")
    s = raw_input()
    if s == 'stop':
        break
    a = A.initiate_dialog_to_get_action(s)
    print response_generator.get_action_sentence(a)
    #r = action_sender.take_action(a)
    #print "RESULT: "+str(r)

#print "testing Generator:"
#while True:
    #s = raw_input()
    #if s == 'stop':
        #break
    #_, form = lex.read_syn_sem(s)
    #token_responses = generator.reverse_parse_semantic_form(form, n=1)
    #print "token responses: "+str(token_responses)

