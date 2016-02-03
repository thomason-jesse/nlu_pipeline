__author__ = 'aishwarya'
# Basically StaticDialogPolicy but with modifications to enable belief 
# tracking

import Action
from Utils import *

class PomdpStaticDialogPolicy:

    def __init__(self):
        pass

    # return an Action object if the DialogState suggests one
    def resolve_state_to_action(self, s):

        a_name = None
        for n in s.user_action_belief:
            if s.user_action_belief[n] == 1:
                a_name = n

        if a_name is None:
            return None

        a_params = []
        for i in range(0, s.action_num_expected_params[a_name]):
            for p in s.user_action_parameters_belief[i]:
                if s.user_action_parameters_belief[i][p] == 1:
                    a_params.append(p)
                    break

        if len(a_params) == s.action_num_expected_params[a_name]:
            return Action.Action(a_name, a_params)

        return None

    # select an action given StaticDialogState
    def select_dialog_action(self, s):

        # confirm action, then patient, then recipient, then extraneous arguments
        a_name, a_max = arg_max(s.user_action_belief)
        unsure = True if a_max < 1 else False

        missing_params = []
        if a_name is not None:
            a_preds = []
            for i in range(0, s.action_num_expected_params[a_name]):
                p_name, p_max = arg_max(s.user_action_parameters_belief[i])
                if p_name is not None:
                    if p_max < 1:
                        unsure = True
                else:
                    missing_params.append(i)
                a_preds.append(p_name)
            a = Action.Action(a_name, a_preds)
            if unsure:
                if (s.previous_action is not None and s.previous_action[0] == "confirm_action" and
                        s.previous_action[1][0] == a):
                    return "repeat_goal", [], Action.Action()
                else:
                    return "confirm_action", [a], a
            elif len(missing_params) > 0:
                return "request_missing_param", [missing_params[0]], a
            else:
                return "repeat_goal", [], Action.Action()

        else:
            return "repeat_goal", [], Action.Action()

