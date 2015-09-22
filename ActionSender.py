#!/usr/bin/env python
__author__ = 'jesse'

import rospy
import actionlib
from bwi_kr_execution.srv import *
from bwi_kr_execution.msg import *


class ActionSender:

    def __init__(self, lex, gen, out):
        self.lexicon = lex
        self.generator = gen
        self.output = out

    def execute_plan_action_client(self, action):

        # convert action into goal
        fluent = AspFluent()
        fluent.name = "not "+action.name
        fluent.variables = action.params[:]
        rule = AspRule()
        rule.body = [fluent]
        goal = ExecutePlanGoal()
        goal.aspGoal = [rule]

        # send goal to system
        try:
            client = actionlib.SimpleActionClient('/action_executor/execute_plan', ExecutePlanAction)
            client.wait_for_server()
            client.send_goal(goal)
            client.wait_for_result()
            return True
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False

    # returns possible groundings for given semantic node
    def take_action(self, action):

        # if action is speaking, use generator to reply
        if action.name == "speak_t" or action.name == "speak_e":
            if type(action.params[0]) is str:
                s = action.params[0]
            elif action.params[0]:
                s = "yes"
            else:
                s = "no"
            form = self.lexicon.read_semantic_form_from_str(s, None, None, [])
            to_say = self.generator.reverse_parse_semantic_form(form, k=3, n=1)
            if len(to_say) > 0:
                self.output.say(" ".join(to_say[0][0]))
                r = True
            else:
                r = False

        # else, take action
        else:
            r = self.execute_plan_action_client(action)

        return r
