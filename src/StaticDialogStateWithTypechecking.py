__author__ = 'aishwarya'

from Knowledge import Knowledge
from Utils import *

class StaticDialogStateWithTypechecking:

    def __init__(self, grounder):

        self.num_user_turns = 0
        self.user_action_belief = {}
        self.action_num_expected_params = {}  # expected number of parameters for seen actions
        self.user_action_parameters_belief = []  # dict for each parameter in order of patient, recipient, location
        self.previous_action = None
        self.knowledge = Knowledge()
        self.grounder = grounder

    def __str__(self):

        s = "num user turns: "+str(self.num_user_turns)+"\n"
        s += "user action belief: "+str(self.user_action_belief)+"\n"
        s += "action num expected params: "+str(self.action_num_expected_params)+"\n"
        s += "user action parameters belief: "+str(self.user_action_parameters_belief)+"\n"
        return s

    def update_requested_user_turn(self):
        self.num_user_turns += 1

    def update_from_failed_parse(self):
        return

    # update state given Action a and SemanticNode parse root p
    def update_from_action(self, a, p):

        # update action belief
        self.user_action_belief[a.name] = 0.9
        for action in self.user_action_belief:
            if action != a.name:
                self.user_action_belief[action] *= 0.5  # decay confidence, similar to IJCAI paper

        # update parameter belief
        for i in range(0, len(a.params)):
            if len(self.user_action_parameters_belief) == i:
                self.user_action_parameters_belief.append({})
            if a.params[i] != "UNK_E":
                if self.param_valid(i, a.params[i]) :
                    self.user_action_parameters_belief[i][a.params[i]] = 0.9
            for param in self.user_action_parameters_belief[i]:
                if param != a.params[i]:
                    self.user_action_parameters_belief[i][param] *= 0.5  # decay confidence, similar to IJCAI paper

        # update expected number of parameters for action
        if p.children is not None:
            self.action_num_expected_params[a.name] = len(p.children)
        else:
            self.action_num_expected_params[a.name] = 0

    # update state given partial Action a and confirmation value t
    def update_from_action_confirmation(self, a, t):

        # update action and parameter beliefs
        if t:
            self.user_action_belief[a.name] = 1
            for i in range(0, len(a.params)):
                if a.params[i] is not None:
                    self.user_action_parameters_belief[i][a.params[i]] = 1
        else:
            self.user_action_belief[a.name] *= 0.5
            for i in range(0, len(a.params)):
                if a.params[i] is not None:
                    self.user_action_parameters_belief[i][a.params[i]] *= 0.5

    def get_max_belief_action(self) :
        inverse = [(value, key) for key, value in self.user_action_belief.items()]
        return max(inverse)[1]

    # update state given a previously missing parameter at action param idx
    def update_from_missing_param(self, param, idx):
        if self.param_valid(idx, param) :
            self.user_action_parameters_belief[idx][param] = 1

    def param_valid(self, idx, param) :
        goal = self.get_max_belief_action()
        param_order = self.knowledge.param_order[goal]
        true_constraints = self.knowledge.true_constraints[goal][param_order[idx]]
        false_constraints = self.knowledge.false_constraints[goal][param_order[idx]]
        for pred in true_constraints :
            if not predicate_holds(pred, param, self.grounder) :
                return False
        for pred in false_constraints :
            if predicate_holds(pred, param, self.grounder) :
                return False
        return True
