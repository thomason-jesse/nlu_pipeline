#!/usr/bin/env python
__author__ = 'rcorona'

import rospy
import sys
import Ontology
import Lexicon
import KBGrounder
import CKYParser
import StaticDialogPolicy
import ActionSender
import Action
from TemplateBasedGenerator import TemplateBasedGenerator
from StaticDialogAgent import StaticDialogAgent
from utils import *

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

action = Action('at', ['l_3516'])
sender = ActionSender(None, None, None)


