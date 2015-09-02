import random
import Action

class DialogAgent:

    def __init__(self, parser, grounder):
        self.parser = parser
        self.grounder = grounder

    def respond_to_utterance(self, u):

        try:
            action = self.get_action_from_utterance(u)
        except SystemError as err:
            return "action from utterance failed with error: '"+str(err.args)+"'"

        return "ACTION: "+str(action)

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

    def get_action_from_utterance(self, u):

        # get response from parser
        n_best_parses = self.parser.parse_expression(u, n=100)
        if len(n_best_parses) == 0:
            raise SystemError("could not parse utterance")
        print n_best_parses
        for i in range(len(n_best_parses)-1, -1, -1):
            parse = n_best_parses[i]
            root = parse[0]
            parse_tree = parse[1]
            print "candidate parse: " + self.parser.print_parse(root)
            print self.parser.print_semantic_parse_result(parse_tree)
            try:
                a_response = self.get_action_from_parse(root)
            except SystemError as err:
                a_response = "failed to ground action"
            print a_response
            print "\n"

        return self.get_action_from_parse(root)

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