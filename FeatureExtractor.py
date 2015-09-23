__author__ = 'jesse'


class FeatureExtractor:
    def __init__(self, ont, lex):
        self.ontology = ont
        self.lexicon = lex

    def increment_dictionary(self, d, idx, inc=1.0):
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
            if ps[1] not in t:
                needle_parts = ps[1].split(' ')
                for np in needle_parts:
                    ta[t.index(np)] = ps[0]
            else:
                ta[t.index(ps[1])] = ps[0]
        else:
            iterate_over = ps if ps[0] is not None and type(ps[0]) is not int else ps[1]
            if iterate_over:
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

        # TODO: count of (token,lexical_entry) chosen at leaf level for ambiguous tokens

        # TODO: do something with the high level action chosen, maybe, or stop passing it in

        to_examine = partial[:] if type(partial) is list else [partial]

        # count the liklihoods of categories appear in parses; roughly corresponds to production rule liklihood
        category_uni = {}
        for p in to_examine:
            if p is None:
                continue
            if type(p) is int:
                p = self.lexicon.semantic_forms[p]
            for cat_idx in p.categories_used:
                self.increment_dictionary(category_uni, [cat_idx], inc=1.0/len(to_examine))
        f_map['category_uni'] = category_uni

        token_surface_pred_co = {}
        token_category_co = {}
        for t in tokens:
            vocab_idx = self.lexicon.surface_forms.index(t)

            # count (token,predicate) pairs between tokens and surface semantic forms chosen for them
            # TODO: the way this is implemented won't work with the generator so think about alternatives
            stack_examine = to_examine[:]
            while len(stack_examine) > 0:
                curr = stack_examine.pop()
                if curr is None:
                    self.increment_dictionary(token_surface_pred_co, [vocab_idx, 'None'])
                elif type(curr) is int:
                    self.increment_dictionary(token_surface_pred_co, [vocab_idx, curr])
                elif type(curr) is list and type(curr[0]) is int:
                    self.increment_dictionary(token_surface_pred_co, [vocab_idx, curr[0]])
                else:
                    if not curr.is_lambda:
                        self.increment_dictionary(token_surface_pred_co, [vocab_idx, curr.idx])
                    if curr.children is not None: stack_examine.extend(curr.children)

            # count (token,category) pairs

            for p in to_examine:
                if p is None:
                    continue
                if type(p) is int:
                    p = self.lexicon.semantic_forms[p]
                for cat_idx in p.categories_used:
                    self.increment_dictionary(token_category_co, [vocab_idx, cat_idx],
                                              inc=1.0/len(to_examine))

        f_map['token_surface_pred_co'] = token_surface_pred_co
        f_map['token_category_co'] = token_category_co

        return f_map
