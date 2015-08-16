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
            if ps[1] in t:
                ta[t.index(ps[1])] = ps[0]
            else:  # for multi-word tokens, assign semantics to each for now (sloppy, ambiguous)
                for sub_token in ps[1].split(' '):
                    ta[t.index(sub_token)] = ps[0]
        else:
            for psc in ps:
                self.read_parse_structure_to_token_assignments(t, ta, psc)

    def extract_feature_map(self, tokens, partial, parse_structure, denotation):
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
        denotation_card = {}
        lambda_card = {}
        lambda_type = {}
        den_type = {}
        for t in tokens:
            vocab_idx = self.lexicon.surface_forms.index(t)

            # count (token,predicate) pairs between tokens and surface semantic form
            to_examine = [p]
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

            # count of (token,|lambda|),(token,type(lambda)),(token,|dens|),(token,type(dens))
            if len(denotation) > 0:
                dl = float(len(denotation))
                dltl = sum([float(len(d[0])) for d in denotation])
                self.increment_dictionary(denotation_card, [vocab_idx], inc=1 / dl)
                for d in denotation:
                    self.increment_dictionary(lambda_card, [vocab_idx], inc=len(d[0]) / dl)
                    for i in range(0, len(d[0])):
                        self.increment_dictionary(lambda_type,
                                                 [vocab_idx, self.ontology.entries[self.ontology.preds.index(d[0][i])]],
                                                 inc=1 / dltl)
                        self.increment_dictionary(den_type, [vocab_idx, str(d[1])], inc=1 / dl)

        f_map['token_surface_pred_co'] = token_surface_pred_co
        f_map['token_denotation_card'] = denotation_card
        f_map['token_lambda_card'] = lambda_card
        f_map['token_lambda_type'] = lambda_type
        f_map['token_den_type'] = den_type

        return f_map
