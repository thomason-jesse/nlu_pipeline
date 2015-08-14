import os,sys
sys.path.append('.') #necessary to import local libraries
import Ontology,Lexicon,FeatureExtractor,LinearLearner,Parser

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

print "instantiating Parser"
parser = Parser.Parser(ont, lex, learner, grounder=None, beam_width=100)

print "reading in data and beginning test"
D = parser.readInPairedUtteranceSemantics(sys.argv[3])
for [x,y] in D:
	print "parsing "+str(x)
	k_best = parser.parseTokens(x)
	correct_found = False
	for ans in k_best:
		if (y.__eq__(ans[0])):
			correct_found = True
			break
	if (correct_found == False):
		print "mismatch on tokens "+str(x)
		print "correct form: "
		print parser.printParse(y)
		print "predicted forms:"
		for ans in k_best:
			print ans[2]
			print parser.printParse(ans[0])
			print parser.printSemanticParseResult(ans[1])
		raise AssertionError("Correct parse was not generated for "+str(x))
