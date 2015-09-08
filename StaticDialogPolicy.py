__author__ = 'jesse'

import Action


class StaticDialogPolicy:

    def __init__(self):
        pass

    # return an Action object if the DialogState s suggests one
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

    # select an action given DialogState s
    def select_dialog_action(self, s):

        return "repeat_goal"
