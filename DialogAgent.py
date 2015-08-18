class DialogAgent:

    def __init__(self, parser, grounder):
        self.parser = parser
        self.grounder = grounder

    def respond_to_utterance(self, u):

        # get response from parser
        n_best_parses = self.parser.parse_expression(u, n=1)
        if len(n_best_parses) == 0:
            return "could not parse utterance"
        parse = n_best_parses[0]
        root = parse[0]
        parse_tree = parse[1]
        print "parse: " + self.parser.print_parse(root)
        print self.parser.print_semantic_parse_result(parse_tree)
        root.set_return_type(self.parser.ontology)  # in case this was not calculated during parsing

        # answer interrogative utterances (true/false)
        if root.return_type == self.parser.ontology.types.index('t'):
            g = self.grounder.groundSemanticNode(root, [], [], [])
            print g  # DEBUG
            if len(g) == 0:
                return "under no circumstances"
            elif len(g) > 1:
                return "under multiple circumstances"
            elif len(g[0][0]) > 0:
                return "under one circumstance"
            else:
                return "yes"

        # answer interrogative utterances (value)
        if root.return_type == self.parser.ontology.types.index('e'):
            g = self.grounder.groundSemanticNode(root, [], [], [])
            print g  # DEBUG
            if len(g) == 0:
                return "there is no such thing"
            elif len(g) > 1:
                return "under different circumstances, it could be "+", ".join([gr[1] for gr in g])
            else:
                return g[0][1]

        # execute action from imperative utterance
        if root.return_type == self.parser.ontology.types.index('a'):
            return "cannot yet take commands"

        # update internal knowledge from declarative utterance
        if root.return_type == self.parser.ontology.types.index('d'):
            return "cannot yet consider declaratives"

        else:
            print "WARNING: unrecognized return type "+str(self.parser.ontology.types[root.return_type])
            return "could not respond to parse of utterance"
