__author__ = 'jesse'

import sys


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

    # treats history as a bag of unordered partial trees, extracts and normalizes features over this whole bag
    def extract_feature_map(self, tokens, parse, action):
        partial = parse[0]
        parse_structure = parse[1]
        history = parse[2]
        f_map = {}

        # calculate the total forms to be examined for normalization
        total_forms = 0
        hist_idx = 0
        while True:
            to_examine = partial[:] if type(partial) is list else [partial]
            total_forms += len(to_examine)
            if history is not None and hist_idx < len(history)-1:
                partial = history[hist_idx][0]
                parse_structure = history[hist_idx][1]
                hist_idx += 1
            else:
                break
        partial = parse[0]
        parse_structure = parse[1]

        token_features = {}
        token_form_features = {}
        form_features = {}
        action_form_features = {}
        action_features = {}
        hist_idx = 0
        while True:

            to_examine = partial[:] if type(partial) is list else [partial]

            # extract token,form features
            if tokens is not None:
                for t_idx in range(0, len(tokens)):
                    vocab_idx = False if tokens[t_idx] is False else self.lexicon.surface_forms.index(tokens[t_idx])

                    # token bigrams
                    d = 'token_bigrams'
                    inc = 1.0 / ((len(tokens)+1) * len(self.lexicon.surface_forms) * len(self.lexicon.surface_forms))
                    if t_idx < len(tokens)-1:
                        next_vocab_idx = False if tokens[t_idx+1] is False else self.lexicon.surface_forms.index(tokens[t_idx+1])
                        self.increment_dictionary(token_features, [d, vocab_idx, next_vocab_idx], inc=inc)

                    # token,predicate bigrams
                    stack_examine = to_examine[:]
                    d = 'token_surface_pred_co'
                    inc = 1.0 / ((len(tokens)+1) * len(self.lexicon.semantic_forms) * total_forms)
                    while len(stack_examine) > 0:
                        curr = stack_examine.pop()
                        if curr is None:
                            self.increment_dictionary(token_form_features, [d, vocab_idx, 'None'], inc=inc)
                            continue
                        if type(curr) is int:
                            curr = self.lexicon.semantic_forms[curr]
                        if type(curr) is str:  # DEBUG
                            print tokens, parse, action  # DEBUG
                            sys.exit()  # DEBUG
                        if not curr.is_lambda:
                            self.increment_dictionary(token_form_features, [d, vocab_idx, curr.idx], inc=inc)
                        if curr.children is not None: stack_examine.extend(curr.children)

                    # token,category bigrams
                    d = 'token_category_co'
                    inc = 1.0 / ((len(tokens)+1) * len(self.lexicon.categories) * total_forms)
                    for p in to_examine:
                        if p is None:
                            continue
                        if type(p) is int:
                            p = self.lexicon.semantic_forms[p]
                        for cat_idx in p.categories_used:
                            self.increment_dictionary(token_form_features, [d, vocab_idx, cat_idx], inc=inc)

            # extract form features
            d = 'category_bigrams'
            inc = 1.0 / (len(self.lexicon.categories) * len(to_examine))
            for i in range(0, len(to_examine)):
                if to_examine[i] is None:
                    cat_i = None
                elif type(to_examine[i]) is int:
                    cat_i = self.lexicon.semantic_forms[to_examine[i]].category
                else:
                    cat_i = to_examine[i].category
                for j in range(i+1, len(to_examine)):
                    if to_examine[j] is None:
                        cat_j = None
                    elif type(to_examine[j]) is int:
                        cat_j = self.lexicon.semantic_forms[to_examine[j]].category
                    else:
                        cat_j = to_examine[j].category
                    self.increment_dictionary(form_features, [d, cat_i, cat_j], inc=inc)

            # extract form, action features
            if action is not None and action.name is not None:
                action_pieces = [action.name]
                action_pieces.extend(action.params)
                for a in action_pieces:
                    if type(a) is bool:
                        a_idx = a
                    else:
                        a_idx = self.ontology.preds.index(a)

                    # action,predicate bigrams
                    stack_examine = to_examine[:]
                    d = 'action_surface_pred_co'
                    inc = 1.0 / (len(tokens) * len(self.lexicon.semantic_forms) * total_forms)
                    while len(stack_examine) > 0:
                        curr = stack_examine.pop()
                        if curr is None:
                            self.increment_dictionary(token_form_features, [d, a_idx, 'None'], inc=inc)
                        elif type(curr) is int:
                            self.increment_dictionary(token_form_features, [d, a_idx, curr], inc=inc)
                        elif type(curr) is list and type(curr[0]) is int:
                            self.increment_dictionary(token_form_features, [d, a_idx, curr[0]], inc=inc)
                        else:
                            if not curr.is_lambda:
                                self.increment_dictionary(token_form_features, [d, a_idx, curr.idx], inc=inc)
                            if curr.children is not None: stack_examine.extend(curr.children)

                    # action,category bigrams
                    d = 'action_category_co'
                    inc = 1.0 / (len(tokens) * len(self.lexicon.categories) * total_forms)
                    for p in to_examine:
                        if p is None:
                            continue
                        if type(p) is int:
                            p = self.lexicon.semantic_forms[p]
                        for cat_idx in p.categories_used:
                            self.increment_dictionary(token_form_features, [d, a_idx, cat_idx], inc=inc)

            # TODO: extract action features

            if history is not None and hist_idx < len(history)-1:
                partial = history[hist_idx][0]
                parse_structure = history[hist_idx][1]
                hist_idx += 1
            else:
                break

        f_map['token_features'] = token_features
        f_map['form_features'] = form_features
        f_map['token_form_features'] = token_form_features
        f_map['action_form_features'] = action_form_features
        f_map['action_features'] = action_features

        return f_map
