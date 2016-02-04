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
import StaticDialogPolicy
import numpy, re, time, rospy
import os, stat
import traceback
import datetime

from TemplateBasedGenerator import TemplateBasedGenerator
from PomdpDialogAgent import PomdpDialogAgent
from StaticDialogAgent import StaticDialogAgent
from Utils import *
from nlu_pipeline.srv import *

from std_msgs.msg import String
from Queue import Queue, Empty
from threading import Lock, Thread

MAX_BUFFER_SIZE = 1024
MAX_WAITING_USERS = 100
MAX_OUTSTANDING_MESSAGES = 1000
MAX_TIMEOUT_TO_ADD_USER = 1 # in seconds

LOGGING_PATH = '../../../../public_html/AMT/log/'
FINAL_ACTION_PATH = '../../../../public_html/AMT/executed_actions/'
USER_LOG_FILE = '../../../../public_html/AMT/log_special/user_list.txt'
ERROR_LOG_FILE = '../../../../public_html/AMT/log_special/errors.txt'
LEXICAL_ADDITION_LOG = '../../../../public_html/AMT/log_special/lexical_addition.txt'

#LOGGING_PATH = '../../../../Trial/AMT/log/'
#FINAL_ACTION_PATH = '../../../../Trial/AMT/executed_actions/'
#USER_LOG = '../../../../Trial/AMT/log_special/user_list.txt'
#ERROR_LOG = '../../../../Trial/AMT/log_special/errors.txt'
#LEXICAL_ADDITION_LOG = '../../../../Trial/AMT/log_special/lexical_addition.txt'

# Fixing the random seed for debugging
numpy.random.seed(4)

class InputFromService:
    def __init__(self, error_log, logfile=None):
        self.service = rospy.Service('dialogue_js_talk', dialogue_js_talk, self.receive_msg)
        self.lock = Lock()
        self.prev_msg = None
        self.logfile = logfile
        self.error_log = error_log
        self.last_get = ''  
        self.lock = Lock()

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
        text = req.js_response
        while True :
            if self.prev_msg is not None :
                success = False
                self.lock.acquire() # Locking is only because service 
                                    # calls are on a separate thread
                try :
                    if self.prev_msg is not None :
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
        return True
    
class OutputToService:
    def __init__(self, input_from_topic, error_log, logfile=None):
        self.service = rospy.Service('dialogue_python_talk', dialogue_python_talk, self.send_msg)
        self.response = None
        self.logfile = logfile  
        self.error_log = error_log  
        self.input_from_topic = input_from_topic
        self.lock = Lock()

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
    def __init__(self) :
        rospy.init_node('dialog_agent_aishwarya')
        self.service = rospy.Service('register_user', register_user, self.on_user_receipt)
        self.user_log = open(USER_LOG_FILE, 'a')
        self.error_log = open(ERROR_LOG_FILE, 'a')
        self.pomdp_agent = None
        self.static_agent = None
        self.current_user = None
        self.lock = Lock()
        self.u_in = None
        self.u_out = None

    def init_static_dialog_agent(self, args) :
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

        load_parser_from_file = False
        if len(args) > 4 :
            if args[4].lower() == 'true' :
                load_parser_from_file = True
                
        if load_parser_from_file :
            parser = load_model('static_parser')
            grounder.parser = parser
            grounder.ontology = parser.ontology
        else :
            print "instantiating Parser"
            parser = Parser.Parser(ont, lex, learner, grounder, beam_width=10, safety=True)

        print "instantiating Generator"
        generator = Generator.Generator(ont, lex, learner, parser, beam_width=sys.maxint, safety=True)

        print "instantiating DialogAgent"
        static_policy = StaticDialogPolicy.StaticDialogPolicy()
        A = StaticDialogAgent(parser, generator, grounder, static_policy, None, None)

        if not load_parser_from_file :
            print "reading in training data"
            D = A.read_in_utterance_action_pairs(args[3])

            if len(args) > 4 and args[4] == "both":
                print "training parser and generator jointly from actions"
                converged = A.jointly_train_parser_and_generator_from_utterance_action_pairs(
                    D, epochs=10, parse_beam=30, generator_beam=10)
            else:
                print "training parser from actions"
                converged = A.train_parser_from_utterance_action_pairs(
                    D, epochs=10, parse_beam=30)

            print "theta: "+str(parser.learner.theta)
            save_model(parser, 'static_parser')
        
        self.static_agent = A

    def init_pomdp_dialog_agent(self, args) :
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

        load_models_from_file = False
        if len(args) > 4 :
            if args[4].lower() == 'true' :
                load_models_from_file = True

        if load_models_from_file :
            parser = load_model('pomdp_parser')
            grounder.parser = parser
            grounder.ontology = parser.ontology
        else :
            print "Instantiating Parser"
            parser = Parser.Parser(ont, lex, learner, grounder, beam_width=10)

        print "Instantiating DialogAgent"
        if load_models_from_file :
            agent = PomdpDialogAgent(parser, grounder, None, None, parse_depth=10, load_policy_from_file=True)
        else :
            agent = PomdpDialogAgent(parser, grounder, None, None, parse_depth=10, load_policy_from_file=False)

        if not load_models_from_file :
            print "reading in data and training parser from actions"
            D = agent.read_in_utterance_action_pairs(args[3])
            converged = agent.train_parser_from_utterance_action_pairs(D, epochs=10, parse_beam=30)
            print "theta: "+str(parser.learner.theta)
            save_model(parser, 'pomdp_parser')
            #print 'Parser ontology : ', parser.ontology.preds

        self.pomdp_agent = agent

    def on_user_receipt(self, req):
        print 'Received user ', req.user_id

        # Try to acquire lock but do not wait
        acquired_lock = self.lock.acquire(False)

        if acquired_lock :
            print 'Acquired lock'
            # No dialog agent running and no concurrent users being added
            success = False
            try :
                if self.current_user is None :
                    print 'New user'
                    # This check is needed because a user may just have 
                    # gotten accepted but their dialogue may not have 
                    # started yet
                    self.current_user = req.user_id
                    success = True
                else :
                    print 'System busy with user ', self.current_user
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
                return success 
        else :
            # The system is currently handling another user
            print 'Could not acquire lock'
            return False

    def launch(self) :
        if self.pomdp_agent is None or self.static_agent is None :
            raise RuntimeError('Agents must be created before launching server')
        
        print 'Dialog agent ready'
        
        user_poll_rate = rospy.Rate(100) # If idle, poll for a new user at 100 Hz
        while not rospy.is_shutdown() :
            if self.current_user is not None :
                # We think there is a new user
                self.lock.acquire()
                try :
                    if self.current_user is not None :
                        # Ideally this check is unnecessary but just in 
                        # case assignments are not atomic
                        logfile = None
                        final_action_log = None
                        logfile = LOGGING_PATH + self.current_user + '.txt'
                        final_action_log = FINAL_ACTION_PATH + self.current_user + '.txt'
                        self.u_in = InputFromService(self.error_log, logfile)
                        self.u_out = OutputToService(self.u_in, self.error_log, logfile)
                        self.pomdp_agent.lexical_addition_log = LEXICAL_ADDITION_LOG + '_pomdp.txt'
                        self.static_agent.lexical_addition_log = LEXICAL_ADDITION_LOG + '_static.txt'
                        
                        # Randomly choose an agent
                        r = numpy.random.random_sample()
                        if r < 0.5 :   
                            self.user_log.write(self.current_user + ',static\n')
                            self.error_log.write(self.current_user + ',static\n') 
                            self.static_agent.input = self.u_in
                            self.static_agent.output = self.u_out
                            self.run_static_dialog(final_action_log)
                        else :
                            self.user_log.write(self.current_user + ',pomdp\n')
                            self.error_log.write(self.current_user + ',pomdp\n')
                            self.pomdp_agent.input = self.u_in
                            self.pomdp_agent.output = self.u_out
                            self.run_pomdp_dialog(final_action_log)       
                except KeyboardInterrupt, SystemExit :
                    pass
                except :
                    error = str(sys.exc_info()[0])
                    self.error_log.write(error + '\n')
                    print traceback.format_exc()
                    self.error_log.write(traceback.format_exc() + '\n\n\n')
                    self.error_log.flush()        
                finally :
                    self.current_user = None 
                        # Free up for next user regardless of whether 
                        # the dialogue finished or crashed
                    self.lock.release()        
                
            user_poll_rate.sleep()

    def run_static_dialog(self, final_action_log=None) :
        self.u_out.say("How can I help?")
        s = self.u_in.get()
        if s == 'stop':
            self.u_out.say("<END/>")
            return
        a = self.static_agent.initiate_dialog_to_get_action(s)
        if a is not None :
            response_generator = TemplateBasedGenerator()
            self.u_out.say(response_generator.get_action_sentence(a, final_action_log) + ' Was this the right action?')
            r = self.u_in.get()
        #u_out.say("Happy to help!")
        self.u_out.say("<END/>")
        
    def run_pomdp_dialog(self, final_action_log=None) :
        self.pomdp_agent.final_action_log = final_action_log
        self.pomdp_agent.first_turn = True
        success = self.pomdp_agent.run_dialog()
        #if not success :
            #u_out.say("I'm sorry I could not help you.")
        #else :
            #u_out.say("Happy to help!")
        self.u_out.say("<END/>")
        
def main(args) :        
    server = DialogueServer()
    server.init_static_dialog_agent(args)
    server.init_pomdp_dialog_agent(args)
    server.launch()

if __name__ == '__main__' :
    main(sys.argv)
