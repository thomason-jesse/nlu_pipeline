#!/usr/bin/env python
__author__ = 'aishwarya'

import sys
import Ontology
import Lexicon
import KBGrounder
import CKYParser
import StaticDialogPolicy
import numpy, re, time, rospy
import os, stat
import traceback
import datetime
import copy

from TemplateBasedGenerator import TemplateBasedGenerator
from PomdpKtdqPolicy import PomdpKtdqPolicy
from PomdpDialogAgent import PomdpDialogAgent
from StaticDialogAgent import StaticDialogAgent
from Knowledge import Knowledge
from utils import *
from nlu_pipeline.srv import *

from std_msgs.msg import String
from Queue import Queue, Empty
from threading import Lock, Thread

MAX_BUFFER_SIZE = 1024
MAX_WAITING_USERS = 100
MAX_OUTSTANDING_MESSAGES = 1000
MAX_TIMEOUT_TO_ADD_USER = 1 # in seconds

MAIN_LOG_PATH = '../../../../public_html/AMT/'
#MAIN_LOG_PATH = ''

LOGGING_PATH = 'log/'
TEXT_LOG_PATH = 'log_text/'
FINAL_ACTION_PATH = 'executed_actions/'
USER_LOG_FILE = 'log_special/users.txt'
MAIN_ERROR_LOG_FILE = 'log_special/errors.txt'
DIALOG_ERROR_LOG_PATH = 'error/'

# Fixing the random seed for debugging
#numpy.random.seed(4)

class InputFromService:
    def __init__(self, user_id,  error_log, logfile=None):
        self.lock = Lock()
        self.js_talk_service = rospy.Service('dialogue_js_talk_' + user_id, dialogue_js_talk, self.receive_msg)
        self.prev_msg = None
        self.logfile = logfile
        self.error_log = error_log
        self.last_get = ''  

    def get(self):
        print 'In get'
        timeout_min = 2
        timeout = time.time() + 60*timeout_min 
        sleeper = rospy.Rate(100)
        while True :
            if time.time() > timeout :
                raise RuntimeError('No reply received on websocket for ' + str(timeout_min) + ' minutes. Timing out.')
            if self.prev_msg is not None :
                text = None
                # The following critical section is purely precautionary
                self.lock.acquire()
                try :
                    text = self.prev_msg
                except KeyboardInterrupt, SystemExit :
                    pass
                except :
                    error = str(sys.exc_info()[0])
                    self.error_log.write(error + '\n')
                    print traceback.format_exc()
                    self.error_log.write(traceback.format_exc() + '\n\n\n')
                    self.error_log.flush()
                finally :
                    self.prev_msg = None
                    self.lock.release()    
                    
                if text is not None :
                    print 'Received: ', text
                    self.last_get = text
                    text = text.lower()
                    regex = re.compile('[\?\.,\;\:]')
                    text = regex.sub('', text)
                    if self.logfile is not None :
                        mode = stat.S_IRWXU | stat.S_IRWXG | stat.S_IRWXO 
                        flags = os.O_CREAT | os.O_APPEND | os.O_WRONLY 
                        fd = os.open(self.logfile, flags, mode)
                        f = os.fdopen(fd, 'a')
                        f.write('USER: ' + text + '\n')
                        f.close()
                        os.chmod(self.logfile, mode)
                    return text 
            sleeper.sleep() 
            
    def receive_msg(self, req) :
        text = req.js_response[:-1]
        print 'Received ', text 
        while True :
            if self.prev_msg is None :
                print 'self.prev_msg is None'
                success = False
                self.lock.acquire() # Locking is only because service 
                                    # calls are on a separate thread
                try :
                    if self.prev_msg is None :
                        # This check is probably unnecessary
                        self.prev_msg = text 
                        success = True
                except KeyboardInterrupt, SystemExit :
                    pass
                except :
                    error = str(sys.exc_info()[0])
                    self.error_log.write(error + '\n')
                    print traceback.format_exc()
                    self.error_log.write(traceback.format_exc() + '\n\n\n')
                    self.error_log.flush()
                finally :
                    self.lock.release()    
                    if success :
                        break
            else :
                print 'self.prev_msg = ', self.prev_msg
        return True
    
class OutputToService:
    def __init__(self, user_id,  input_from_topic, error_log, logfile=None):
        self.lock = Lock()
        self.python_talk_service = rospy.Service('dialogue_python_talk_' + user_id, dialogue_python_talk, self.send_msg)
        self.response = None
        self.logfile = logfile  
        self.error_log = error_log  
        self.input_from_topic = input_from_topic

    def say(self, response):
        if self.logfile is not None :
            mode = stat.S_IRWXU | stat.S_IRWXG | stat.S_IRWXO 
            flags = os.O_CREAT | os.O_APPEND | os.O_WRONLY
            fd = os.open(self.logfile, flags, mode)
            f = os.fdopen(fd, 'a')
            f.write('ROBOT: ' + response + '\n')
            f.close()
            os.chmod(self.logfile, mode)
            
        self.lock.acquire()
        try :
            self.response = response
            print 'Saying: ', response
        except KeyboardInterrupt, SystemExit :
            pass
        except :
            error = str(sys.exc_info()[0])
            self.error_log.write(error + '\n')
            print traceback.format_exc()
            self.error_log.write(traceback.format_exc() + '\n\n\n')
            self.error_log.flush()
        finally :
            self.lock.release()

    def send_msg(self, req) :
        msg_poller = rospy.Rate(100)
        while True :
            response = None
            if self.response is not None :
                self.lock.acquire()
                try :
                    response = self.input_from_topic.last_get + '\n' + self.response
                    self.response = None
                except KeyboardInterrupt, SystemExit :
                    pass
                except :
                    error = str(sys.exc_info()[0])
                    self.error_log.write(error + '\n')
                    print traceback.format_exc()
                    self.error_log.write(traceback.format_exc() + '\n\n\n')
                    self.error_log.flush()
                finally :
                    self.lock.release()
            
            if response is not None :
                return response
                
            msg_poller.sleep()

class DialogueServer :
    def __init__(self, args) :
        print 'args = ', args, '\n\n\n\n'
        if len(args) < 4 :
            print 'Usage ', args[0], ' ont_file lex_file parser_train_pairs_file [load_models_from_file=true/false]'
        
        rospy.init_node('dialog_agent_aishwarya')
        
        self.user_log = open(MAIN_LOG_PATH + USER_LOG_FILE, 'a')
        self.error_log = open(MAIN_LOG_PATH + MAIN_ERROR_LOG_FILE, 'a')
        self.started_users = set()
        
        print "reading in Ontology"
        ont = Ontology.Ontology(args[1])
        print "predicates: " + str(ont.preds)
        print "types: " + str(ont.types)
        print "entries: " + str(ont.entries)
        self.ont = ont
        
        print "reading in Lexicon"
        lex = Lexicon.Lexicon(ont, args[2])
        print "surface forms: " + str(lex.surface_forms)
        print "categories: " + str(lex.categories)
        print "semantic forms: " + str(lex.semantic_forms)
        print "entries: " + str(lex.entries)
        self.lex = lex
        
        self.parser_train_file = args[3]
        
        self.load_models_from_file = False
        if len(args) > 4 :
            if args[4].lower() == 'true' :
                print 'Going to load from file'
                self.load_models_from_file = True
        
        self.lock = Lock()
        self.service = rospy.Service('register_user', register_user, self.on_user_receipt)
        
    def create_static_dialog_agent(self) :
        ont = copy.deepcopy(self.ont)
        lex = copy.deepcopy(self.lex)
        
        print "instantiating KBGrounder"
        grounder = KBGrounder.KBGrounder(ont)
             
        if self.load_models_from_file :
            parser = load_model('only_parser_parser')
        else :
            parser = CKYParser.CKYParser(ont, lex, use_language_model=True)
            # Set parser hyperparams to best known values for training
            parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
            parser.max_new_senses_per_utterance = 3  # max number of new word senses that can be induced on a training example
            parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
            parser.max_hypothesis_categories_for_unknown_token_beam = 3  # for unknown token, max syntax categories tried
            # Train parser
            d = parser.read_in_paired_utterance_semantics(self.parser_train_file)
            converged = parser.train_learner_on_semantic_forms(d, 10, reranker_beam=10)
            save_model(parser, 'only_parser_parser')
            
        # Set parser hyperparams to best known values for test time
        parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
        parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
        parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
        parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried

        grounder.parser = parser
        grounder.ontology = parser.ontology
  
        static_policy = StaticDialogPolicy.StaticDialogPolicy()
        static_agent = StaticDialogAgent(parser, grounder, static_policy, None, None)
        static_agent.retrain_parser = False
        return static_agent

    def create_pomdp_dialog_agent(self, parser_file=None) :
        ont = copy.deepcopy(self.ont)
        lex = copy.deepcopy(self.lex)
        
        print "instantiating KBGrounder"
        grounder = KBGrounder.KBGrounder(ont)
        
        if self.load_models_from_file :
            parser = load_model(parser_file)
        else :
            parser = CKYParser.CKYParser(ont, lex, use_language_model=True)
            # Set parser hyperparams to best known values for training
            parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
            parser.max_new_senses_per_utterance = 3  # max number of new word senses that can be induced on a training example
            parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
            parser.max_hypothesis_categories_for_unknown_token_beam = 3  # for unknown token, max syntax categories tried
            # Train parser
            d = parser.read_in_paired_utterance_semantics(self.parser_train_file)
            converged = parser.train_learner_on_semantic_forms(d, 10, reranker_beam=10)
            save_model(parser, parser_file)
            
        # Set parser hyperparams to best known values for test time
        parser.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
        parser.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
        parser.max_cky_trees_per_token_sequence_beam = 1000  # for tokenization of an utterance, max cky trees considered
        parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried

        grounder.parser = parser
        grounder.ontology = parser.ontology
  
        print "Instantiating DialogAgent"
        if self.load_models_from_file :
            policy = load_model('ktdq_policy')
            policy.untrained = False
            policy.training = False
        else :
            knowledge = Knowledge()
            policy = PomdpKtdqPolicy(knowledge)
            policy.untrained = True
            policy.training = True
        agent = PomdpDialogAgent(parser, grounder, policy, None, None)
        agent.retrain_parser = False
        return agent

    # Receives a request from a new user, creates an appropriate dialog 
    # agent and starts dialog on a separate thread
    # Since calls to this function involve writes to come common files
    # like user_log, it involves locking
    def on_user_receipt(self, req):
        #print 'Received user ', req.user_id
        self.error_log.write('Received user ' + str(req.user_id) + '\n')
        self.error_log.flush()
        #x = raw_input()
        
        # Try to acquire lock but do not wait
        acquired_lock = self.lock.acquire(False)

        if acquired_lock :
            print 'Acquired lock'
            if req.user_id not in self.started_users :
                success = False
                try :
                    self.handle_user(req.user_id)   
                    print 'Returned'
                    self.started_users.add(req.user_id)     
                    success = True
                except KeyboardInterrupt, SystemExit :
                    raise
                except :
                    error = str(sys.exc_info()[0])
                    self.error_log.write(error + '\n')
                    print traceback.format_exc()
                    self.error_log.write(traceback.format_exc() + '\n\n\n')
                    self.error_log.flush()        
                finally :
                    self.lock.release()
                    self.user_log.flush()
                    self.error_log.flush()
                    return success 
            else :
                self.lock.release()
                self.user_log.flush()
                self.error_log.write('Repeated user ID ' + req.user_id + '\n\n')
                self.error_log.flush()
                return False
        else :
            # The system is currently handling another user
            print 'Could not acquire lock'
            self.user_log.flush()
            self.error_log.write('Could not acquire lock\n\n')
            self.error_log.flush()
            return False

    def handle_user(self, user_id) :
        self.error_log.write('Handling user ' + str(user_id) + '\n')
        self.error_log.flush()
        text_log = MAIN_LOG_PATH + TEXT_LOG_PATH + user_id + '.txt'
        
        u_in = InputFromService(user_id, self.error_log, text_log)
        u_out = OutputToService(user_id, u_in, self.error_log, text_log)
        try :
            final_action_log = MAIN_LOG_PATH + FINAL_ACTION_PATH + user_id + '.txt'
            log = MAIN_LOG_PATH + LOGGING_PATH + user_id + '.pkl'
            error_log = MAIN_LOG_PATH + DIALOG_ERROR_LOG_PATH + user_id + '.txt'
            
            # Randomly choose an agent
            #r = numpy.random.random_sample()
            r = 0.1
            if r < 1.0 / 3 :   
                print 'Only parser learning'
                self.user_log.write(user_id + ',only_parser\n')
                self.error_log.write(user_id + ',only_parser\n') 
                static_agent = self.create_static_dialog_agent()
                static_agent.input = u_in
                static_agent.output = u_out
                static_agent.final_action_log = final_action_log
                static_agent.dialog_objects_logfile = log
                static_agent.log_header = 'only_parser_learning'
                static_agent.error_log = open(error_log, 'a')
                
                dialog_thread = Thread(target=self.run_static_dialog, args=((static_agent,)))
                dialog_thread.daemon = True
                dialog_thread.start()
            elif r < 2.0 / 3 :
                print 'Only dialog learning'
                self.user_log.write(user_id + ',only_dialog\n')
                self.error_log.write(user_id + ',only_dialog\n')
                pomdp_agent = self.create_pomdp_dialog_agent('only_dialog_parser')
                pomdp_agent.input = u_in
                pomdp_agent.output = u_out
                pomdp_agent.final_action_log = final_action_log
                pomdp_agent.dialog_objects_logfile = log
                pomdp_agent.log_header = 'only_dialog_learning'
                pomdp_agent.error_log = open(error_log, 'a')
                
                dialog_thread = Thread(target=self.run_pomdp_dialog, args=((pomdp_agent,)))
                dialog_thread.daemon = True
                dialog_thread.start()
            else :
                print 'Dialog and parser learning'
                self.user_log.write(user_id + ',both\n')
                self.error_log.write(user_id + ',both\n')
                pomdp_agent = self.create_pomdp_dialog_agent('both_parser')
                pomdp_agent.input = u_in
                pomdp_agent.output = u_out
                pomdp_agent.final_action_log = final_action_log
                pomdp_agent.dialog_objects_logfile = log
                pomdp_agent.log_header = 'both_parser_and_dialog_learning'
                pomdp_agent.error_log = open(error_log, 'a')
                
                dialog_thread = Thread(target=self.run_pomdp_dialog, args=((pomdp_agent,)))
                dialog_thread.daemon = True
                dialog_thread.start()
        except (KeyboardInterrupt, SystemExit) :
            raise
        except :
            # Kill services
            u_in.shutdown()
            u_out.shutdown()
            
            error = str(sys.exc_info()[0])
            self.error_log.write(error + '\n')
            print traceback.format_exc()
            self.error_log.write(traceback.format_exc() + '\n\n\n')
            self.error_log.flush()

    def launch(self) :
        print 'Waiting for users...'
        rospy.spin()

    def run_static_dialog(self, agent) :
        print 'Starting static agent...'
        try :
            agent.output.say("How can I help?")
            s = agent.input.get()
            if s == 'stop':
               agent.output.say("<END/>")
               return
            a = agent.initiate_dialog_to_get_action(s)
            agent.output.say("<END/>")
        except KeyboardInterrupt, SystemExit :
            pass
        except :
            error = str(sys.exc_info()[0])
            self.error_log.write(error + '\n')
            print traceback.format_exc()
            self.error_log.write(traceback.format_exc() + '\n\n\n')
            self.error_log.flush()  
        
    def run_pomdp_dialog(self, agent) :
        print 'Starting POMDP agent ...'
        try :
            agent.first_turn = True
            success = agent.run_dialog()
            agent.output.say("<END/>")
        except KeyboardInterrupt, SystemExit :
            pass
        except :
            error = str(sys.exc_info()[0])
            self.error_log.write(error + '\n')
            print traceback.format_exc()
            self.error_log.write(traceback.format_exc() + '\n\n\n')
            self.error_log.flush()  
        
def main(args) :        
    server = DialogueServer(args)
    server.launch()

if __name__ == '__main__' :
    main(sys.argv)
