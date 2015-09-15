__author__ = 'jesse'


class FeatureExtractor:
    def __init__(self, ont, lex):
        self.ontology = ont
        self.lexicon = lex

    def increment_dictionary(self, d, idx, inc=1):
        dr = d
        for i in range(0, len(idx)):
            if idx[i] not in dr:
                if i == len(idx) - 1:
                    dr[idx[i]] = 0
                else:
                    dr[idx[i]] = {}
            if i == len(idx) - 1:
                dr[idx[i]] += inc
            else:
                dr = dr[idx[i]]

    def read_parse_structure_to_token_assignments(self, t, ta, ps):
        if len(ps) == 2 and type(ps[1]) is str:
            ta[t.index(ps[1])] = ps[0]
        else:
            iterate_over = ps if ps[0] is not None and type(ps[0]) is not int else ps[1]
            for psc in iterate_over:
                if psc is not False:  # in generation, can have None child that hasn't been expanded/matched yet
                    self.read_parse_structure_to_token_assignments(t, ta, psc)

    def extract_feature_map(self, tokens, parse, action):
        partial = parse[0]
        parse_structure = parse[1]
        f_map = {}

        # count features related to number of trees in partial and presence of None assignments
        sf = {}
        if len(tokens) != 0:
            sf['trees_per_token'] = len(parse_structure) / float(len(tokens))
            sf['prop_trees'] = 1.0 / len(parse_structure)
            token_assignments = [None for i in range(0, len(tokens))]
            self.read_parse_structure_to_token_assignments(tokens, token_assignments, parse_structure)
            sf['None_per_token'] = sum([1 if p is None else 0 for p in token_assignments]) / float(len(tokens))
        f_map['sf'] = sf

        # count of (token,lexical_entry) chosen at leaf level for ambiguous tokens
        # TODO

        # count of (rule,syntax,syntax) combinations used to form surface semantics
        # TODO

        token_surface_pred_co = {}
        for t in tokens:
            vocab_idx = self.lexicon.surface_forms.index(t)

            # count (token,predicate) pairs between tokens and surface semantic form
            to_examine = partial[:] if type(partial) is list else [partial]
            while len(to_examine) > 0:
                curr = to_examine.pop()
                if curr is None:
                    self.increment_dictionary(token_surface_pred_co, [vocab_idx, 'None'])
                elif type(curr) is int:
                    self.increment_dictionary(token_surface_pred_co, [vocab_idx, curr])
                elif type(curr) is list and type(curr[0]) is int:
                    self.increment_dictionary(token_surface_pred_co, [vocab_idx, curr[0]])
                else:
                    if not curr.is_lambda:
                        self.increment_dictionary(token_surface_pred_co, [vocab_idx, curr.idx])
                    if curr.children is not None: to_examine.extend(curr.children)

        f_map['token_surface_pred_co'] = token_surface_pred_co

        return f_map
