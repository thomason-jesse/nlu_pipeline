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
import StaticDialogPolicy
import numpy, re
import socket

from PomdpStaticDialogPolicy import PomdpStaticDialogPolicy
from PomdpDialogAgent import PomdpDialogAgent
from Utils import *

MAX_BUFFER_SIZE = 1024
MAX_OUTSTANDING_REQUESTS = 10

class InputFromSocket:
    def __init__(self, socket):
        self.socket = socket

    def get(self):
        try :
            text = self.socket.recv(MAX_BUFFER_SIZE)
            text = text.lower()
            regex = re.compile('[\?\.,\;\:]')
            text = regex.sub('', text)
            print 'Received: ', text 
            print 'Press enter'
            x = raw_input()
            return text 
        except socket.error :
            print 'Error receiving'
            return '<ERROR/>'    
        

class OutputToSocket:
    def __init__(self, socket):
        self.socket = socket

    def say(self, s):
        try :
            print 'Sending: ', s
            print 'Press enter'
            x = raw_input()
            self.socket.send(s)
        except socket.error :
            print 'Error sending'
            return '<ERROR/>'


# Fixing the random seed for debugging
#numpy.random.seed(4)

def init_dialog_agent(args) :
    print "Reading in Ontology"
    ont = Ontology.Ontology(args[1])
    print "predicates: " + str(ont.preds)
    print "types: " + str(ont.types)
    print "entries: " + str(ont.entries)

    print "Reading in Lexicon"
    lex = Lexicon.Lexicon(ont, args[2])
    print "surface forms: " + str(lex.surface_forms)
    print "categories: " + str(lex.categories)
    print "semantic forms: " + str(lex.semantic_forms)
    print "entries: " + str(lex.entries)

    print "Instantiating Feature Extractor"
    f_extractor = FeatureExtractor.FeatureExtractor(ont, lex)

    print "Instantiating Linear Learner"
    learner = LinearLearner.LinearLearner(ont, lex, f_extractor)

    print "Instantiating KBGrounder"
    grounder = KBGrounder.KBGrounder(ont)

    print "Instantiating Parser"
    parser = Parser.Parser(ont, lex, learner, grounder, beam_width=10)
    parser = load_model('parser')
    grounder.parser = parser
    grounder.ontology = parser.ontology

    print "Instantiating DialogAgent"
    agent = PomdpDialogAgent(parser, grounder, None, None)

    #print "reading in data and training parser from actions"
    #D = agent.read_in_utterance_action_pairs(args[3])
    #converged = agent.train_parser_from_utterance_action_pairs(D, epochs=10, parse_beam=30)
    #print "theta: "+str(parser.learner.theta)
    #save_model(parser, 'parser')
    #print 'Parser ontology : ', parser.ontology.preds

    return agent

def listen(agent) :
    print 'Creating server socket'
    # create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    host = 'localhost'
    port = 9999                                           
    server_socket.bind((host, port))                                  
    server_socket.listen(MAX_OUTSTANDING_REQUESTS)                                           
    print '\n\nDialog agent listening on ' + str(host) + ':' + str(port)

    while True :
        # establish a connection
        client_socket,addr = server_socket.accept()      
        print("Received connection from %s" % str(addr))
        u_in = InputFromSocket(client_socket)
        u_out = OutputToSocket(client_socket)
        agent.input = u_in
        agent.output = u_out
        run_dialog(agent, u_in, u_out)
        client_socket.close()

def run_dialog(agent, u_in, u_out) :
    agent.first_turn = True
    success = agent.run_dialog()
    if not success :
        u_out.say("I'm sorry I could not help you.")
    else :
        u_out.say("Happy to help!")
        
if __name__ == '__main__' :
    agent = init_dialog_agent(sys.argv)
    listen(agent)
