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
LOGGING_PATH = 'log/'

class UserManager :
    def __init__(self) :
        rospy.init_node('dialog_agent_aishwarya')
        self.user_queue = Queue(MAX_WAITING_USERS) # Using this as it is thread safe
        self.prev_user = None
        rospy.Subscriber("new_user_topic", String, self.add_to_queue)
        self.lock = Lock()
    
    def get_next_user(self) :
        try :
            # No need for lock as Queue is thread safe
            user = self.user_queue.get_nowait() 
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
                    self.user_queue.put(user) 
                        # This will actually block till the put  gets done
                        # But as it is in the critical section, there should
                        # be no competing puts
                    self.prev_user = msg.data
                    print 'Processed'
            except KeyboardInterrupt, SystemExit :
                pass
            except :
                print 'Error : ', sys.exc_info()[0]
            finally :
                self.lock.release()
                print 'Released lock'

# This class has a lot of similarities to the previous one so may have 
# been worth making an abstract class for them
class InputFromTopic:
    def __init__(self, user):
        topic_name = 'js_pub_' + user
        self.sub = rospy.Subscriber(topic_name, String, self.add_to_queue)
        self.msg_queue = Queue(MAX_OUTSTANDING_MESSAGES) # Using this as it is thread safe
        self.lock = Lock()
        self.prev_msg = None
        self.logging_enabled = False    
        self.current_user = user
        self.last_get = ''  

    def get(self):
        print 'In get'
        timeout_min = 2
        timeout = time.time() + 60*timeout_min 
        while True :
            if time.time() > timeout :
                raise RuntimeError('No reply received on websocket for ' + str(timeout_min) + ' minutes. Timing out.')
            try :
                text = self.msg_queue.get_nowait() 
                print 'Received: ', text
                self.last_get = text
                text = text.lower()
                regex = re.compile('[\?\.,\;\:]')
                text = regex.sub('', text)
                #print 'Press enter'
                #x = raw_input()
                if self.logging_enabled :
                    filename = LOGGING_PATH + self.current_user
                    f = open(filename, 'a')
                    f.write('USER: ' + text + '\n')
                    f.close()
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
                    self.msg_queue.put(text) 
                        # This will actually block till the put gets 
                        # done but as it is in the critical section,  
                        # there should be no competing puts
                    self.prev_msg = msg.data
            except KeyboardInterrupt, SystemExit :
                pass
            except :
                print 'Error : ', sys.exc_info()[0]
            finally :
                self.lock.release()    
    
    def __del__(self) :
        self.sub.unregister()

class OutputToTopic:
    def __init__(self, user, input_from_topic):
        self.response = None
        topic_name =  'python_pub_' + user
        self.logging_enabled = False    
        self.current_user = user  
        self.pub = rospy.Publisher(topic_name, String, queue_size=MAX_OUTSTANDING_MESSAGES)
        self.msg = None
        self.seq_no = 0
        self.input_from_topic = input_from_topic
        self.publish_thread = Thread(target=self.publish, args=())
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def say(self, response):
        if self.logging_enabled :
            filename = LOGGING_PATH + self.current_user
            f = open(filename, 'a')
            f.write('ROBOT: ' + response + '\n')
            f.close()
        print 'Going to publish: ', response
        self.msg = str(self.seq_no) + '|' + self.input_from_topic.last_get + '|' + response
        self.seq_no += 1

    def publish(self) :
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.msg is not None :
                self.pub.publish(self.msg)
            r.sleep()

    def __del__(self) :
        self.pub.unregister()

# Fixing the random seed for debugging
#numpy.random.seed(4)

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
    
    while True :
        user = user_manager.get_next_user()    
        if user is not None :
            # There is actually a user
            try :
                print 'Starting communication with user', user
                user_manager.lock.acquire()
                # Making this a critical section otherwise sometimes the
                # dialogue hangs 
                try :
                    u_in = InputFromTopic(user)
                    u_out = OutputToTopic(user, u_in)
                except :
                    raise
                finally :
                    user_manager.lock.release()
                
                # Randomly choose an agent
                r = numpy.random.random_sample()
                if r < 0.5 :    
                    static_agent.input = u_in
                    static_agent.output = u_out
                    run_static_dialog(static_agent, u_in, u_out)
                else :
                    pomdp_agent.input = u_in
                    pomdp_agent.output = u_out
                    run_pomdp_dialog(pomdp_agent, u_in, u_out)
                print 'Waiting for a new user '
            except RuntimeError as e :
                print 'Error : ', str(e)
                print 'Waiting for a new user '

def run_static_dialog(agent, u_in, u_out) :
    u_out.say("How can I help?")
    s = u_in.get()
    if s == 'stop':
        return
    a = agent.initiate_dialog_to_get_action(s)
    if a is not None :
        response_generator = TemplateBasedGenerator()
        u_out.say(response_generator.get_action_sentence(a) + ' Was this the right action?')
        r = u_in.get()
    #u_out.say("Happy to help!")
    u_out.say("<END/>")
    
def run_pomdp_dialog(agent, u_in, u_out) :
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
