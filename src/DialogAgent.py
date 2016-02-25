__author__ = 'jesse'

import random
import copy
import sys
import re
import Action
import StaticDialogState

class DialogAgent:

    def __init__(self, parser, grounder, policy, u_input, output, parse_depth=10, max_parse_beam=20):
        self.parser = parser
        self.grounder = grounder
        self.parse_depth = parse_depth
        self.max_parse_beam = max_parse_beam
        self.state = None
        self.policy = policy
        self.input = u_input
        self.output = output

        # define action names and functions they correspond to
        self.dialog_actions = ["repeat_goal", "confirm_action", "request_missing_param"]
        self.dialog_action_functions = [self.request_user_initiative, self.confirm_action, self.request_missing_param]

    # initiate a new dialog with the agent with initial utterance u
    def initiate_dialog_to_get_action(self, u):
        # print "Function call succeeded"         

        self.state = StaticDialogState.StaticDialogState()
        # print "Created state"
        self.update_state_from_user_initiative(u)
        # print "Updated state"

        # select next action from state
        action = None
        while action is None:
            print "dialog state: "+str(self.state)  # DEBUG
            action = self.policy.resolve_state_to_action(self.state)

            # if the state does not clearly define a user goal, take a dialog action to move towards it
            if action is None:
                dialog_action, dialog_action_args = self.policy.select_dialog_action(self.state)
                # print "Going to take action: ", str(dialog_action)
                if dialog_action not in self.dialog_actions:
                    sys.exit("ERROR: unrecognized dialog action '"+dialog_action+"' returned by policy for state "+str(self.state))
                self.state.previous_action = [dialog_action, dialog_action_args]
                self.dialog_action_functions[self.dialog_actions.index(dialog_action)](dialog_action_args)

        return action

    # request the user repeat their original goal
    def request_user_initiative(self, args):
        self.output.say("Can you restate your question or command?")
        u = self.input.get()
        self.update_state_from_user_initiative(u)

    # request a missing action parameter
    def request_missing_param(self, args):
        idx = args[0]
        # not sure whether reverse parsing could frame this
        if idx == 0:
            theme = "agent"
        elif idx == 1:
            theme = "patient"
        elif idx == 2:
            theme = "location"
        else:
            sys.exit("ERROR: unrecognized theme idx "+str(idx))
        self.output.say("What is the " + theme + " of the action you'd like me to take?")
        u = self.input.get()
        self.update_state_from_missing_param(u, idx)

    # update state after asking user to provide a missing parameter
    def update_state_from_missing_param(self, u, idx):
        self.state.update_requested_user_turn()

        # get n best parses for confirmation
        #n_best_parses = self.parser.parse_expression(u, n=self.parse_depth)
        n_best_parses = self.get_n_best_parses(u)

        # try to digest parses to confirmation
        success = False
        for i in range(0, len(n_best_parses)):
            # print self.parser.print_parse(n_best_parses[i][0])  # DEBUG
            # print self.parser.print_semantic_parse_result(n_best_parses[i][1])  # DEBUG
            g = self.grounder.groundSemanticNode(n_best_parses[i][0].node, [], [], [])
            answer = self.grounder.grounding_to_answer_set(g)
            if len(answer) == 1:
                success = True
                self.state.update_from_missing_param(answer[0], idx)
                break
        if not success:
            self.state.update_from_failed_parse()

    # confirm a (possibly partial) action
    def confirm_action(self, args):
        a = args[0]
        # with reverse parsing, want to confirm(a.name(a.params))
        # for now, just use a.name and a.params raw
        self.output.say("I should take action " + a.name+" involving " +
                        ','.join([str(p) for p in a.params if p is not None]) + "?")
        c = self.input.get()
        self.update_state_from_action_confirmation(c, a)

    # update state after asking user to confirm a (possibly partial) action
    def update_state_from_action_confirmation(self, c, a):
        self.state.update_requested_user_turn()

        # get n best parses for confirmation
        #n_best_parses = self.parser.parse_expression(c, n=self.parse_depth)
        n_best_parses = self.get_n_best_parses(c)

        # try to digest parses to confirmation
        success = False
        for i in range(0, len(n_best_parses)):
            # print self.parser.print_parse(n_best_parses[i][0])  # DEBUG
            # print self.parser.print_semantic_parse_result(n_best_parses[i][1])  # DEBUG
            if n_best_parses[i][0].node.idx == self.parser.ontology.preds.index('yes'):
                success = True
                self.state.update_from_action_confirmation(a, True)
            elif n_best_parses[i][0].node.idx == self.parser.ontology.preds.index('no'):
                success = True
                self.state.update_from_action_confirmation(a, False)
        if not success:
            self.state.update_from_failed_parse()

    # update state after asking user-initiative (open-ended) question about user goal
    def update_state_from_user_initiative(self, u):
        self.state.update_requested_user_turn()

        # get n best parses for utterance
        #print 'n_best_parses - ', n_best_parses #DEBUG
        n_best_parses = self.get_n_best_parses(u)

        # try to digest parses to action request
        success = False
        for i in range(0, len(n_best_parses)):
            #print self.parser.print_parse(n_best_parses[i][0])  # DEBUG
            #print self.parser.print_semantic_parse_result(n_best_parses[i][1])  # DEBUG
            success = self.update_state_from_action_parse(n_best_parses[i][0].node)
            if success:
                break

        # no parses could be resolved into actions for state update
        if not success:
            # print "Has to recover from failed parse"
            self.state.update_from_failed_parse()

    # get dialog state under assumption that utterance is an action
    def update_state_from_action_parse(self, p):
        # get action, if any, from the given parse
        try:
            p_action = self.get_action_from_parse(p)
        except SystemError:
            p_action = None
        if p_action is not None:
            print 'Here'
            self.state.update_from_action(p_action, p)
            return True

        # if this failed, try again allowing missing lambdas to become UNK without token ties
        # print "Going to retry parsing"
        UNK_root = copy.deepcopy(p)
        curr = UNK_root
        heading_lambdas = []
        if curr.is_lambda and curr.is_lambda_instantiation:
            heading_lambdas.append(curr.lambda_name)
            curr = curr.children[0]
        if curr.children is not None:
            for i in range(0, len(curr.children)):
                if (curr.children[i].is_lambda and curr.children[i].lambda_name in heading_lambdas
                        and curr.type == self.parser.ontology.types.index('e')):
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
        f = open(fname, 'r')
        f_lines = f.readlines()
        pairs = []
        for i in range(0,len(f_lines), 4):
            t = self.parser.tokenize(f_lines[i].strip())
            cat, r = self.parser.lexicon.read_syn_sem(f_lines[i+1].strip())
            a_str = f_lines[i+2].strip()
            a_name, a_params_str = a_str.strip(')').split('(')
            a_params = a_params_str.split(',')
            for j in range(0, len(a_params)):
                if a_params[j] == "True":
                    a_params[j] = True
                elif a_params[j] == "False":
                    a_params[j] = False
            pairs.append([t, r, Action.Action(a_name, a_params)])
        f.close()
        return pairs

    def train_parser_from_utterance_action_pairs(self, pairs, epochs=10, parse_beam=10):
        for e in range(0, epochs):
            print "training epoch "+str(e)
            random.shuffle(pairs)
            train_data = []
            num_correct = 0
            for t, r, a in pairs:
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
                        a_candidate = self.get_action_from_parse(n_best_parses[i][0].node)
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
                    train_data.append([t, n_best_parses[0], a_chosen, t, n_best_parses[i], a])
                else:
                    num_correct += 1
            print "\t"+str(num_correct)+"/"+str(len(pairs))+" top choices"
            if num_correct == len(pairs):
                print "WARNING: training converged at epoch "+str(e)+"/"+str(epochs)
                return True
            self.parser.learner.learn_from_actions(train_data)
        return False

    def get_n_best_parses(self, response, num_parses_needed=None) :
        if len(response) == 0 :
            return []
        if num_parses_needed is None :
            num_parses_needed = self.parse_depth
        # Parser expects a space between 's and the thing it is applied 
        # to, for example "alice 's" rather than "alice's"
        response = re.sub("'s", " 's", response)
        parse_generator = self.parser.most_likely_cky_parse(response)
        num_parses_obtained = 0
        num_parses_examined = 0
        parses = list()
        for (parse, score, _) in parse_generator :
            num_parses_examined += 1
            if num_parses_examined == self.max_parse_beam :
                break
            if parse is None :
                break
            if parse.node.is_lambda and parse.node.is_lambda_instantiation :
                # Lambda headed parses are very unlikely to be correct. Drop them
                continue
            top_level_category = self.parser.lexicon.categories[parse.node.category]
            if top_level_category not in ['M', 'NP', 'C'] :
                # M - imperative (full action), C - confirmation, NP - noun phrase for params
                continue
            print 'parse = ', self.parser.print_parse(parse.node, show_category=True), ', score = ', score 
            parses.append((parse, score))
            num_parses_obtained += 1
            if num_parses_obtained == num_parses_needed :
                break
        #x = raw_input()
        return parses 

    def get_action_from_parse(self, root):
	# print "Inside get_action_from_parse"

        print "parse to get action from: " + self.parser.print_parse(root)  # DEBUG

        root.set_return_type(self.parser.ontology)  # in case this was not calculated during parsing

        # execute action from imperative utterance
        if root.return_type == self.parser.ontology.types.index('a'):
            # assume for now that logical connectives do not operate over actions (eg. no (do action a and action b))
            # ground action arguments
            action = self.parser.ontology.preds[root.idx]
            print "action: "+action  # DEBUG
            g_args = []
            # print "Going to enter loop"
            for arg in root.children:
                print 'Grounding ', self.parser.print_parse(arg, show_category=True)
                g = self.grounder.groundSemanticNode(arg, [], [], [])
                print "arg: "+str(g)  # DEBUG
                answer = self.grounder.grounding_to_answer_set(g)
                if len(answer) == 0:
                    # print "Single answer found"
                    if action == "speak_t":
                        g_args.append(False)
                    elif action == "speak_e":
                        g_args.append(None)
                    else:
                        raise SystemError("action argument unrecognized. arg: '"+str(answer)+"'")
                elif len(answer) > 1:
                    raise SystemError("multiple interpretations of action argument renders command ambiguous. arg: '"+str(answer)+"'")
                else:
                    # print "No answer found"
                    if action == "speak_t":
                        g_args.append(True)
                    else:
                        g_args.append(answer[0])
            # print "args: "+str(g_args)  # DEBUG
            return Action.Action(action, g_args)

        else:
            raise SystemError("cannot get action from return type "+str(self.parser.ontology.types[root.return_type]))
