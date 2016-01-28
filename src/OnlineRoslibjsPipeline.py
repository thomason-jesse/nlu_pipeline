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

from TemplateBasedGenerator import TemplateBasedGenerator
from PomdpStaticDialogPolicy import PomdpStaticDialogPolicy
from PomdpDialogAgent import PomdpDialogAgent
from StaticDialogAgent import StaticDialogAgent
from Utils import *

from std_msgs.msg import String
from Queue import Queue, Empty
from threading import Lock, Thread

MAX_BUFFER_SIZE = 1024
MAX_WAITING_USERS = 100
MAX_OUTSTANDING_MESSAGES = 1000
MAX_TIMEOUT_TO_ADD_USER = 1 # in seconds

#LOGGING_PATH = 'src/nlu_pipeline/src/log/'
#FINAL_ACTION_PATH = 'src/nlu_pipeline/src/executed_action/'
#USER_LOG = 'src/nlu_pipeline/src/log_special/user_list'
#LEXICAL_ADDITION_LOG = 'src/nlu_pipeline/src/log_special/lexical_addition'

LOGGING_PATH = '../../../../public_html/AMT/log/'
FINAL_ACTION_PATH = '../../../../public_html/AMT/executed_actions/'
USER_LOG = '../../../../public_html/AMT/log_special/user_list.txt'
ERROR_LOG = '../../../../public_html/AMT/log_special/errors.txt'
LEXICAL_ADDITION_LOG = '../../../../public_html/AMT/log_special/lexical_addition.txt'

# Fixing the random seed for debugging
numpy.random.seed(4)

class UserManager :
    def __init__(self) :
        rospy.init_node('dialog_agent_aishwarya')
        self.user_queue = Queue(MAX_WAITING_USERS) # Using this as it is thread safe
        self.prev_user = None
        rospy.Subscriber("new_user_topic", String, self.add_to_queue)
        self.listen_thread = Thread(target=self.listen_for_users, args=())
        self.listen_thread.daemon = True
        self.listen_thread.start()
        self.lock = Lock()
    
    def listen_for_users(self) :
        rospy.spin()
    
    def get_next_user(self) :
        try :
            # No need for lock as Queue is thread safe
            user = self.user_queue.get(True, 0.1) 
            return user
        except Empty :
            return None  
    
    def add_to_queue(self, msg) :
        if self.prev_user != msg.data :
            print 'Received ', msg.data
            self.lock.acquire() # Locking is necessary because otherwise 
                                # the same user could get added twice by
                                # competing callbacks
            print 'Acquired lock'
            try :
                if self.prev_user != msg.data :
                    # Check has to be done again because a competing 
                    # callback could have just added the same user 
                    user = msg.data
                    self.user_queue.put(user, True, MAX_TIMEOUT_TO_ADD_USER) 
                        # Ideally this should not need a timeout as it 
                        # is in a critical section and hence should not 
                        # suffer from competing puts
                    self.prev_user = msg.data
                    print 'Processed'
            except KeyboardInterrupt, SystemExit :
                pass
            except :
                error = str(sys.exc_info()[0])
                error_log.write(error + '\n')
                print traceback.format_exc()
                error_log.write(traceback.format_exc() + '\n\n\n')
                error_log.flush()
            finally :
                self.lock.release()
                print 'Released lock'

# This class has a lot of similarities to the previous one so may have 
# been worth making an abstract class for them
class InputFromTopic:
    def __init__(self, user, logfile=None):
        topic_name = 'js_pub_' + user
        self.sub = rospy.Subscriber(topic_name, String, self.add_to_queue)
        self.msg_queue = Queue(MAX_OUTSTANDING_MESSAGES) # Using this as it is thread safe
        self.lock = Lock()
        self.prev_msg = None
        self.logfile = logfile
        self.current_user = user
        self.last_get = ''  
        self.lock = Lock()
        self.listen_thread = Thread(target=self.listen_for_messages, args=())
        self.listen_thread.daemon = True
        self.listen_thread.start()
    
    def listen_for_messages(self) :
        rospy.spin()

    def get(self):
        print 'In get'
        timeout_min = 2
        timeout = time.time() + 60*timeout_min 
        while True :
            if time.time() > timeout :
                raise RuntimeError('No reply received on websocket for ' + str(timeout_min) + ' minutes. Timing out.')
            try :
                text = self.msg_queue.get(True, 0.1) 
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
            except Empty :
                pass  
            
    def add_to_queue(self, msg) :
        #print 'Received ', msg.data, 'self.prev_msg = ', self.prev_msg
        if '|' not in msg.data :
            print 'Malformed message: ', msg.data
        elif self.prev_msg != msg.data :
            self.lock.acquire() # Locking is necessary because otherwise 
                                # the same message could get added twice 
                                # by competing callbacks
            try :
                if self.prev_msg != msg.data :
                    # Check has to be done again because a competing 
                    # callback could have just added the same message 
                    text = msg.data.split('|')[1]
                    self.msg_queue.put(text, True, 1) 
                        # This will actually block till the put gets 
                        # done but as it is in the critical section,  
                        # there should be no competing puts
                    self.prev_msg = msg.data
            except KeyboardInterrupt, SystemExit :
                pass
            except :
                error = str(sys.exc_info()[0])
                error_log.write(error + '\n')
                print traceback.format_exc()
                error_log.write(traceback.format_exc() + '\n\n\n')
                error_log.flush()
            finally :
                self.lock.release()    
    
    def __del__(self) :
        self.sub.unregister()
        
    def close(self) :
        if self.sub is not None :
            self.sub.unregister()

class OutputToTopic:
    def __init__(self, user, input_from_topic, logfile=None):
        self.response = None
        topic_name =  'python_pub_' + user
        self.logfile = logfile    
        self.current_user = user  
        #self.pub = rospy.Publisher(topic_name, String, queue_size=MAX_OUTSTANDING_MESSAGES)
        self.pub = rospy.Publisher(topic_name, String, queue_size=1, tcp_nodelay=True)
            # The rospy documentation says that a queue size of 1 should
            # be fine for 10 Hz if you're not sedning a burst of messages
            # This will prevent a backlog of messages to be cleared in  
            # case the connection is slow which often happens on Wifi
        self.msg = None
        self.seq_no = 0
        self.input_from_topic = input_from_topic
        self.publish_thread = Thread(target=self.publish, args=())
        self.publish_thread.daemon = True
        self.publish_thread.start()
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
        print 'Going to publish: ', response
        self.lock.acquire()
        try :
            self.msg = str(self.seq_no) + '|' + self.input_from_topic.last_get + '|' + response
            self.seq_no += 1
        except KeyboardInterrupt, SystemExit :
            pass
        except :
            error = str(sys.exc_info()[0])
            error_log.write(error + '\n')
            print traceback.format_exc()
            error_log.write(traceback.format_exc() + '\n\n\n')
            error_log.flush()
        finally :
            self.lock.release()

    def publish(self) :
        print 'Publish thread created \n\n\n'
        r = rospy.Rate(50) # 10hz
        while not rospy.is_shutdown():
            if self.msg is not None :
                self.pub.publish(self.msg)
            r.sleep()

    def __del__(self) :
        self.pub.unregister()
        
    def close(self) :
        self.msg = None
        if self.pub is not None :
            self.pub.unregister()

def init_static_dialog_agent(args) :
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
    
    return A

def init_pomdp_dialog_agent(args) :
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

    return agent

def start(pomdp_agent, static_agent) :
    # Set up the user manager which waits for users
    user_manager = UserManager()
    print 'Dialog agent ready'
    
    user_log = open(USER_LOG, 'a')
    error_log = open(ERROR_LOG, 'a')
    pomdp_agent.lexical_addition_log = LEXICAL_ADDITION_LOG + '_pomdp.txt'
    static_agent.lexical_addition_log = LEXICAL_ADDITION_LOG + '_static.txt'
    
    wait_time = 0
    u_in  = None
    u_out = None
    
    while True :
        #print 'Checking for user'
        try :
            user = user_manager.get_next_user()    
            if user is not None :
                wait_time = 0
                # There is actually a user
                try :
                    print 'Starting communication with user', user
                    if u_in is not None :
                        u_in.close()
                    if u_out is not None :
                        u_out.close()
                    user_manager.lock.acquire()
                    # Making this a critical section otherwise sometimes the
                    # dialogue hangs 
                    logfile = None
                    final_action_log = None
                    try :
                        logfile = LOGGING_PATH + user + '.txt'
                        final_action_log = FINAL_ACTION_PATH + user + '.txt'
                        u_in = InputFromTopic(user, logfile)
                        u_out = OutputToTopic(user, u_in, logfile)
                    except :
                        raise
                    finally :
                        user_manager.lock.release()
                    
                    # Randomly choose an agent
                    r = numpy.random.random_sample()
                    if r < 0.5 :   
                        user_log.write(user + ',static\n')
                        error_log.write(user + ',static\n') 
                        static_agent.input = u_in
                        static_agent.output = u_out
                        run_static_dialog(static_agent, u_in, u_out, final_action_log)
                    else :
                        user_log.write(user + ',pomdp\n')
                        error_log.write(user + ',pomdp\n')
                        pomdp_agent.input = u_in
                        pomdp_agent.output = u_out
                        run_pomdp_dialog(pomdp_agent, u_in, u_out, final_action_log)
                    print 'Waiting for a new user '
                    user_log.flush()
                except RuntimeError as e :
                    error = str(e)
                    error_log.write(error + '\n\n\n')
                    print 'Waiting for a new user '
                    user_log.flush()
                    error_log.flush()
            else :
                # Increase wait time before checking for user again so that 
                # user queue does not get stuck servciing only get requests
                if wait_time <= 1.0 :
                    wait_time += 0.1
            rospy.spinonce()
            time.sleep(wait_time)
        except KeyboardInterrupt, SystemExit :
            raise
        except :
            error = str(sys.exc_info()[0])
            error_log.write(error + '\n')
            print traceback.format_exc()
            error_log.write(traceback.format_exc() + '\n\n\n')
            error_log.flush()
                
    user_log.close()
    error_log.close()

def run_static_dialog(agent, u_in, u_out, final_action_log=None) :
    u_out.say("How can I help?")
    s = u_in.get()
    if s == 'stop':
        
        u_out.say("<END/>")
        return
    a = agent.initiate_dialog_to_get_action(s)
    if a is not None :
        response_generator = TemplateBasedGenerator()
        u_out.say(response_generator.get_action_sentence(a, final_action_log) + ' Was this the right action?')
        r = u_in.get()
    #u_out.say("Happy to help!")
    u_out.say("<END/>")
    
def run_pomdp_dialog(agent, u_in, u_out, final_action_log=None) :
    agent.final_action_log = final_action_log
    agent.first_turn = True
    success = agent.run_dialog()
    #if not success :
        #u_out.say("I'm sorry I could not help you.")
    #else :
        #u_out.say("Happy to help!")
    u_out.say("<END/>")
        
if __name__ == '__main__' :
    static_agent = init_static_dialog_agent(sys.argv)
    pomdp_agent = init_pomdp_dialog_agent(sys.argv)
    start(pomdp_agent, static_agent)  
