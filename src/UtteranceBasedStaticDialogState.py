__author__ = 'aishwarya'

from StaticDialogState import StaticDialogState
from Knowledge import Knowledge
from utils import *

# This maintains the same state information as StaticDialogState but
# updates using Utterance objects rather than strings or Action objects
class UtteranceBasedStaticDialogState(StaticDialogState):

    def __init__(self):
        StaticDialogState.__init__(self)
        self.knowledge = Knowledge()
        
    # Update state given an Utterance u that specifies a complete action 
    # Assumes typechecking has already been done on the Utterance u
    def update_from_action(self, u):
        print 'In update_from_action'
        if u.referring_goal is None :
            return
    
        # update action belief
        self.user_action_belief[u.referring_goal] = 0.9
        for action in self.user_action_belief:
            if action != u.referring_goal:
                self.user_action_belief[action] *= 0.5  # decay confidence, similar to IJCAI paper

        # update parameter belief
        param_order = self.knowledge.param_order[u.referring_goal]
        for i in range(0, len(param_order)):
            if len(self.user_action_parameters_belief) == i:
                self.user_action_parameters_belief.append({})
            if get_dict_val(u.referring_params, param_order[i]) is not None :
                self.user_action_parameters_belief[i][u.referring_params[param_order[i]]] = 0.9
            for param in self.user_action_parameters_belief[i]:
                if param != get_dict_val(u.referring_params, param_order[i]) :
                    self.user_action_parameters_belief[i][param] *= 0.5  # decay confidence, similar to IJCAI paper

        # update expected number of parameters for action
        self.action_num_expected_params[u.referring_goal] = len(param_order)

    # Update state given a SystemAction m and Utterance u which is an
    # affirmation or denial
    def update_from_action_confirmation(self, m, u):
        goal = m.referring_goal
        if goal is None :
            return
        if u.action_type == 'affirm' :
            self.user_action_belief[goal] = 1
            param_order = self.knowledge.param_order[goal]
            for i in range(0, len(param_order)) :
                param_value = get_dict_val(m.referring_params, param_order[i])
                if param_value is not None :
                    self.user_action_parameters_belief[i][param_value] = 1
        else :
            self.user_action_belief[goal] *= 0.5
            param_order = self.knowledge.param_order[goal]
            for i in range(0, len(param_order)) :
                param_value = get_dict_val(m.referring_params, param_order[i])
                if param_value is not None :
                    self.user_action_parameters_belief[i][param_value] *= 0.5

    def get_max_belief_action(self) :
        inverse = [(value, key) for key, value in self.user_action_belief.items()]
        return max(inverse)[1]

    # Update state given a SystemAction m and Utterance u which is of
    # type inform_param
    def update_from_missing_param(self, m, u):
        goal = m.referring_goal
        param_name = m.extra_data[0]
        param_value = get_dict_val(u.referring_params, param_name)
        if goal is not None and param_name is not None and param_value is not None:
            idx = self.knowledge.param_order[goal].index(param_name)
            self.user_action_parameters_belief[idx][param_value] = 1

    # Update state given a SystemAction m and teh first Utterance u in 
    # u_list 
    # This is a generic method which prevents the need of figuring out
    # which of the above update method to call
    # This also avoids updation if the utterance type does not match 
    # what is expected
    def update(self, m, u_list, grounder) :
        print 'In update'
        if len(u_list) > 0 :
            u = u_list[0]
            if m.action_type == 'repeat_goal' :
                if u.action_type == 'inform_full' :
                    self.update_from_action(u)
            elif m.action_type == 'confirm_action' :
                if u.action_type == 'affirm' or u.action_type == 'deny' :
                    self.update_from_action_confirmation(m, u)
            elif m.action_type == 'request_missing_param' :
                if u.action_type == 'inform_param' :
                    self.update_from_missing_param(m, u)
            
        
