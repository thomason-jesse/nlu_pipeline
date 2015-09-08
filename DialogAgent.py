__author__ = 'jesse'

import random
import copy
import sys
import Action
import DialogState


class DialogAgent:

    def __init__(self, parser, grounder, policy, u_input, output, parse_depth=10):
        self.parser = parser
        self.grounder = grounder
        self.parse_depth = parse_depth
        self.state = None
        self.policy = policy
        self.input = u_input
        self.output = output

        # define action names and functions they correspond to
        self.dialog_actions = ["repeat_goal"]
        self.dialog_action_functions = [self.request_user_initiative]

    # initiate a new dialog with the agent with initial utterance u
    def initiate_dialog_to_get_action(self, u):

        self.state = DialogState.DialogState()
        self.update_state_from_user_initiative(u)

        # select next action from state
        action = None
        while action is None:
            print "dialog state: "+str(self.state)  # DEBUG
            action = self.policy.resolve_state_to_action(self.state)

            # if the state does not clearly define a user goal, take a dialog action to move towards it
            if action is None:
                dialog_action = self.policy.select_dialog_action(self.state)
                if dialog_action not in self.dialog_actions:
                    sys.exit("ERROR: unrecognized dialog action '"+dialog_action+"' returned by policy for state "+str(self.state))
                self.dialog_action_functions[self.dialog_actions.index(dialog_action)]()

        return action

    # request the user repeat their original goal
    def request_user_initiative(self):
        self.output.say("Can you restate your question or command?")
        u = self.input.get()
        self.update_state_from_user_initiative(u)

    # update state after asking user-initiative (open-ended) question about user goal
    def update_state_from_user_initiative(self, u):
        self.state.update_requested_user_turn()

        # get n best parses for utterance
        n_best_parses = self.parser.parse_expression(u, n=self.parse_depth)

        # try to digest parses to action request to achieve an initial state
        success = False
        for i in range(0, len(n_best_parses)):
            success = self.update_state_from_action_parse(n_best_parses[i][0])
            if success:
                break

        # no parses could be resolved into actions for state update
        if not success:
            self.state.update_from_failed_parse()

    # get dialog state under assumption that utterance is an action
    def update_state_from_action_parse(self, p):

        # get action, if any, from the given parse
        try:
            p_action = self.get_action_from_parse(p)
        except SystemError:
            p_action = None
        if p_action is not None:
            self.state.update_from_action(p_action, p)
            return True

        # if this failed, try again allowing missing lambdas to become UNK without token ties
        UNK_root = copy.deepcopy(p)
        curr = UNK_root
        heading_lambdas = []
        if curr.is_lambda and curr.is_lambda_instantiation:
            heading_lambdas.append(curr.lambda_name)
            curr = curr.children[0]
        if curr.children is not None:
            for i in range(0, len(curr.children)):
                if curr.children[i].is_lambda and curr.children[i].lambda_name in heading_lambdas:
                    curr.children[i] = self.parser.create_unk_node()
        try:
            p_unk_action = self.get_action_from_parse(UNK_root)
        except SystemError:
            p_unk_action = None
        if p_unk_action is not None:
            self.state.update_from_action(p_unk_action, UNK_root)
            return True

        # parse could not be resolved into an action with which to update state
        return False

    def read_in_utterance_action_pairs(self, fname):
        f = open(fname,'r')
        f_lines = f.readlines()
        pairs = []
        for i in range(0,len(f_lines),3):
            t = self.parser.tokenize(f_lines[i].strip())
            a_str = f_lines[i+1].strip()
            a_name, a_params_str = a_str.strip(')').split('(')
            a_params = a_params_str.split(',')
            for j in range(0, len(a_params)):
                if a_params[j] == "True":
                    a_params[j] = True
                elif a_params[j] == "False":
                    a_params[j] = False
            pairs.append([t, Action.Action(a_name, a_params)])
        f.close()
        return pairs

    def train_parser_from_utterance_action_pairs(self, pairs, epochs=10, parse_beam=10):
        for e in range(0, epochs):
            print "training epoch "+str(e)
            random.shuffle(pairs)
            train_data = []
            num_correct = 0
            for t, a in pairs:
                n_best_parses = self.parser.parse_tokens(t, n=parse_beam)
                if len(n_best_parses) == 0:
                    print "WARNING: no parses found for tokens "+str(t)
                    continue
                a_chosen = None
                a_candidate = None
                correct_found = False
                for i in range(0, len(n_best_parses)):
                    try:
                        # print "parse: " + self.parser.print_parse(n_best_parses[i][0])  # DEBUG
                        # print self.parser.print_semantic_parse_result(n_best_parses[i][1])  # DEBUG
                        a_candidate = self.get_action_from_parse(n_best_parses[i][0])
                        # print "candidate: "+str(a_candidate)  # DEBUG
                    except SystemError:
                        a_candidate = Action.Action()
                    if i == 0:
                        a_chosen = a_candidate
                    if a_candidate.__eq__(a):
                        correct_found = True
                        self.parser.add_genlex_entries_to_lexicon_from_partial(n_best_parses[i][1])
                        break  # the action matches gold and this parse should be used in training
                if not correct_found:
                    print "WARNING: could not find correct action '"+str(a)+"' for tokens "+str(t)
                if a_chosen != a_candidate:
                    train_data.append([t, n_best_parses[0], a_chosen, n_best_parses[i], a])
                else:
                    num_correct += 1
            print "\t"+str(num_correct)+"/"+str(len(pairs))+" top choices"
            if num_correct == len(pairs):
                print "WARNING: training converged at epoch "+str(e)+"/"+str(epochs)
                return True
            self.parser.learner.learn_from_actions(train_data)
        return False

    def get_action_from_parse(self, root):

        # print "parse to get action from: " + self.parser.print_parse(root)  # DEBUG

        root.set_return_type(self.parser.ontology)  # in case this was not calculated during parsing

        # execute action from imperative utterance
        if root.return_type == self.parser.ontology.types.index('a'):
            # assume for now that logical connectives do not operate over actions (eg. no (do action a and action b))
            # ground action arguments
            action = self.parser.ontology.preds[root.idx]
            # print "action: "+action  # DEBUG
            g_args = []
            for arg in root.children:
                g = self.grounder.groundSemanticNode(arg, [], [], [])
                # print "arg: "+str(g)  # DEBUG
                answer = self.grounder.grounding_to_answer_set(g)
                if len(answer) == 0:
                    if action == "speak_t":
                        g_args.append(False)
                    elif action == "speak_e":
                        g_args.append(None)
                    else:
                        raise SystemError("action argument unrecognized. arg: '"+str(answer)+"'")
                elif len(answer) > 1:
                    raise SystemError("multiple interpretations of action argument renders command ambiguous. arg: '"+str(answer)+"'")
                else:
                    if action == "speak_t":
                        g_args.append(True)
                    else:
                        g_args.append(answer[0])
            # print "args: "+str(g_args)  # DEBUG
            return Action.Action(action, g_args)

        else:
            raise SystemError("cannot get action from return type "+str(self.parser.ontology.types[root.return_type]))