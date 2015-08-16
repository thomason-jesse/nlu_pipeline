#!/usr/bin/env python

import os,sys,rospy
import Ontology,Lexicon,FeatureExtractor,LinearLearner,Parser,KBGrounder

print "reading in Ontology"
ont = Ontology.Ontology(sys.argv[1])
print "predicates: "+str(ont.preds)
print "types: "+str(ont.types)
print "entries: "+str(ont.entries)

print "reading in Lexicon"
lex = Lexicon.Lexicon(ont, sys.argv[2])
print "surface forms: "+str(lex.surface_forms)
print "categories: "+str(lex.categories)
print "semantic forms: "+str(lex.semantic_forms)
print "entries: "+str(lex.entries)

print "instantiating Feature Extractor"
f_extractor = FeatureExtractor.FeatureExtractor(ont, lex)

print "instantiating Linear Learner"
learner = LinearLearner.LinearLearner(ont, lex, f_extractor)

print "instantiating KBGrounder"
grounder = KBGrounder.KBGrounder(ont)

print "instantiating Parser"
parser = Parser.Parser(ont, lex, learner, grounder, beam_width=100)

print "reading in data and training parser"
D = parser.read_in_paired_utterance_denotation(sys.argv[3])
parser.train_learner_on_denotations(D, epochs=10)
print parser.learner.theta

while (True):
	print "\nsentence to be parsed:"
	s = raw_input()
	if (s == 'stop'): break
	print "running parser on '"+s+"'"
	k_best = parser.parse_expression(s,n=10)
	print "best "+str(len(k_best))+" parses found:\n"
	for [p,spr,s] in k_best:
		print str(s)+":\nparse: "+parser.print_parse(p)+"\nSemanticNode:\n"+str(p)
		print "Parse Tree:\n"+parser.print_semantic_parse_result(spr)+"\n"
		print "running grounder on parse..."
		print str(grounder.groundSemanticNode(p,[],[],[]))+"\n"
