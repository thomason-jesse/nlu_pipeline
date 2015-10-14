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

class InputFromKeyboard:
    def __init__(self):
        pass

    def get(self):
        return raw_input()

class InputFromSpeech:
    def __init__(self):
        import ctypes

        if True: #TODO Allow path specification is None:
            self.libHandle = ctypes.CDLL("./src/bwi_common/bwi_dialogue/src/nlu_pipeline/speech/speechRecognizer.so")
        else:
            self.libHandle = ctypes.CDLL(path)

        #Initialize Sphinx. 
        self.libHandle.sphinx_init()
        self.libHandle.initMic()

        self.numbers = {"one": '1', "two": '2', "three": '3', "four": '4', "five": '5',
                        "six": '6', "seven": '7', "eight": '8', "nine": '9'}

    def __del__(self):
        self.libHandle.closeMic()
        self.libHandle.sphinx_close()

    def postProcess(self, words):
        utterance = ""

        for word in words:
            if word in self.numbers:
                utterance += self.numbers[word]
            else:
                utterance += ' ' + word + ' '

        return utterance

    def get(self):
        result =  self.getHypString(self.getNBest(1)[0]).split()

        utterance = self.postProcess(result)

        print utterance

        return utterance

    def record(self):
        self.libHandle.sphinx_n_best_m(1); 

    def recordToggle(self):
        #Initializes pygame window to read spacebar presses. 
        pygame.init()
        pygame.display.set_mode((1,1))

        running = True

        print "\nPress [SPACEBAR] to record"

        while running:
            pygame.event.pump()
            keys = pygame.key.get_pressed()

            if keys[pygame.K_SPACE]:
                print "Recording"

                #Starts recording from mic to file. 
                self.libHandle.startRecord()

                #Loops while user is holding space bar. 
                while keys[pygame.K_SPACE]:
                    pygame.event.pump()
                    keys = pygame.key.get_pressed()

                #Stops recording when they release it. 
                self.libHandle.stopRecord()

                running = False
            
        pygame.quit()

    def getNBest(self, n):

        #Creates threads for recording. 
        thread1 = threading.Thread(target = self.record)
        thread2 = threading.Thread(target = self.recordToggle)

        #Starts thread execution. 
        thread1.start()
        thread2.start()

        #Waits for recording threads to finish. 
        thread1.join()
        thread2.join()

        #Opens up results to begin reading from them. 
        results = open("results.txt", "r")

        #Makes a list of each hypothesis, containing string, confidence, etc.
        resultList = [line.strip('\n') for line in results]

        hypotheses = [result.split(":") for result in resultList]

        return hypotheses

    def getHypString(self, hypothesis):
        return hypothesis[0]

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
generator = Generator.Generator(ont, lex, learner, parser, beam_width=sys.maxint, safety=True, linear_iter_adjust=10)
print "testing Generator:"
while True:
    s = raw_input()
    if s == 'stop':
        break
    _, form = lex.read_syn_sem(s)
    token_responses = generator.reverse_parse_semantic_form(form, n=1, c=1)
    print "token responses: "+str(token_responses)

print "instantiating DialogAgent"
u_in = InputFromSpeech()
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
