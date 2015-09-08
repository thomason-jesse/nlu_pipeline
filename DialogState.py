class DialogState:

    def __init__(self):

        self.num_user_turns = 0
        self.user_action_belief = {}
        self.action_num_expected_params = {}  # expected number of parameters for seen actions
        self.user_action_parameters_belief = []  # dict for each parameter in order of patient, recipient, location

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
        self.user_action_belief[a.name] = 1
        for action in self.user_action_belief:
            if action != a.name:
                self.user_action_belief[action] *= 0.5  # decay confidence, similar to IJCAI paper

        # update parameter belief
        for i in range(0, len(a.params)):
            if len(self.user_action_parameters_belief) == i:
                self.user_action_parameters_belief.append({})
            if a.params[i] != "UNK_E":
                self.user_action_parameters_belief[i][a.params[i]] = 1
            for param in self.user_action_parameters_belief[i]:
                if param != a.params[i]:
                    self.user_action_parameters_belief[i][param] *= 0.5  # decay confidence, similar to IJCAI paper

        # update expected number of parameters for action
        if p.children is not None:
            self.action_num_expected_params[a.name] = len(p.children)
        else:
            self.action_num_expected_params[a.name] = 0