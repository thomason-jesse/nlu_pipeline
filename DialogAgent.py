class DialogAgent:

    def __init__(self, parser, grounder):
        self.parser = parser
        self.grounder = grounder

    def respond_to_utterance(self, u):

        # get response from parser
        n_best_parses = self.parser.parse_expression(u, n=100)
        if len(n_best_parses) == 0:
            return "could not parse utterance"
        for i in range(len(n_best_parses)-1, -1, -1):
            parse = n_best_parses[i]
            root = parse[0]
            parse_tree = parse[1]
            print "parse: " + self.parser.print_parse(root)
            print self.parser.print_semantic_parse_result(parse_tree)
        root.set_return_type(self.parser.ontology)  # in case this was not calculated during parsing

        # answer interrogative utterances (true/false)
        if root.return_type == self.parser.ontology.types.index('t'):
            g = self.grounder.groundSemanticNode(root, [], [], [])
            print g  # DEBUG
            answer = self.grounder.grounding_to_answer_set(g)
            if len(answer) == 0:
                return "under no circumstances"
            elif len(answer) > 1:
                return "under multiple circumstances"
            else:
                return "yes"

        # answer interrogative utterances (value)
        elif root.return_type == self.parser.ontology.types.index('e'):
            g = self.grounder.groundSemanticNode(root, [], [], [])
            print g  # DEBUG
            answer = self.grounder.grounding_to_answer_set(g)
            if len(answer) == 0:
                return "there is no such thing"
            elif len(answer) > 1:
                return "under different circumstances, it could be "+", ".join(answer)
            else:
                return answer[0]

        # execute action from imperative utterance
        elif root.return_type == self.parser.ontology.types.index('a'):
            # assume for now that logical connectives do not operate over actions (eg. no (do action a and action b))
            # ground action arguments
            action = self.parser.ontology.preds[root.idx]
            print "action: "+action
            g_args = []
            for arg in root.children:
                g = self.grounder.groundSemanticNode(arg, [], [], [])
                print "arg: "+str(g)  # DEBUG
                answer = self.grounder.grounding_to_answer_set(g)
                if len(answer) == 0:
                    return "could not recognize action argument"
                elif len(answer) > 1:
                    return "multiple interpretations of action argument renders command ambiguous"
                else:
                    g_args.append(answer[0])
            return "ACTION: "+action+"("+",".join(g_args)+")"

        # update internal knowledge from declarative utterance
        elif root.return_type == self.parser.ontology.types.index('d'):
            return "cannot yet consider declaratives"

        else:
            print "WARNING: unrecognized return type "+str(self.parser.ontology.types[root.return_type])
            return "could not respond to parse of utterance"
