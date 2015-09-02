import sys
import copy
import random
import SemanticNode


class Parser:
    def __init__(self, ont, lex, learner, grounder, beam_width=10):
        self.ontology = ont
        self.lexicon = lex
        self.learner = learner
        self.grounder = grounder
        self.beam_width = beam_width
        self.new_adjectives = [] # cleared before every parsing job

    # print a SemanticNode as a string using the known ontology
    def print_parse(self, p, show_category=False):
        if p is None: return "NONE"
        elif show_category and p.category is not None:
            s = self.lexicon.compose_str_from_category(p.category) + " : "
        else:
            s = ''
        if p.is_lambda:
            if p.is_lambda_instantiation:
                s += "lambda " + str(p.lambda_name) + ":" + self.ontology.compose_str_from_type(p.type) + "."
            else:
                s += str(p.lambda_name)
        else:
            s += self.ontology.preds[p.idx]
        if p.children is not None:
            s += '(' + ','.join([self.print_parse(c) for c in p.children]) + ')'
        return s

    # print a full parse tree using the ontology and internal structure
    def print_semantic_parse_result(self, spr):
        s = ''
        for sp in spr:
            s += self.print_semantic_parse_tree(sp) + "\n"
        return s

    def print_semantic_parse_tree(self, sp, d=0):
        s = ''
        for i in range(0, d): s += '-\t'
        if type(sp) is not tuple:
            if type(sp[0]) is int:
                s += self.print_parse(self.lexicon.semantic_forms[sp[0]])
            else:
                s += str(self.tokens_of_parse_tree(sp))
            for i in range(0, len(sp[1])):
                s += '\n' + self.print_semantic_parse_tree(sp[1][i], d=d+1)
        else:
            if sp[0] is None:
                s += "NONE -: " + sp[1]
            else:
                s += self.print_parse(self.lexicon.semantic_forms[sp[0]]) + " -: " + sp[1]
        return s

    def tokens_of_parse_tree(self, sp):
        if type(sp[1]) is str:
            return sp[1]
        else:
            return [self.tokens_of_parse_tree(m) for m in sp[1]]

    # read in data set of form utterance/denotation\n\n...
    def read_in_paired_utterance_denotation(self, fname):
        D = []
        f = open(fname, 'r')
        f_lines = f.readlines()
        f.close()
        i = 0
        while i < len(f_lines):
            tokens = self.tokenize(f_lines[i].strip())
            denotation_strs = f_lines[i + 1].strip().split(';')
            denotation = [self.str_to_list(ds.strip()) for ds in denotation_strs]
            D.append([tokens, denotation])
            i += 3
        return D

    def str_to_list(self, s):
        if s[0] == '[' and s[-1] == ']':
            b = 0
            if len(s) > 2:
                for idx in range(1, len(s) - 1):
                    if s[idx] == '[':
                        b += 1
                    elif s[idx] == ']':
                        b -= 1
                    elif b == 0 and s[idx] == ',':
                        break
                return [self.str_to_list(s[1:idx].strip().strip("'")), self.str_to_list(s[idx + 1:-1].strip().strip("'"))]
            else:
                return []
        elif s == 'True':
            return True
        else:
            return s

    # take in a data set D=(x,d) for x tokens and d correct denotations and update the learner's parameters
    def train_learner_on_denotations(self, D, epochs=10, k=None, n=None):
        if k is None: k = self.beam_width
        if n is None: n = self.beam_width
        for e in range(0, epochs):
            print "epoch " + str(e)  # DEBUG
            T = []
            correct = 0
            for [x, d] in D:
                n_best_parses = self.parse_tokens(x, k, n)
                if len(n_best_parses) == 0:
                    print "WARNING: could not find valid parse for tokens"+str(x)
                    continue
                highest_scoring_parse = n_best_parses[0]
                correct_denotation_parse = None
                for i in range(0, len(n_best_parses)):
                    print "candidate parse: "+self.print_parse(n_best_parses[i][0])  # DEBUG
                    candidate_denotation = self.grounder.groundSemanticNode(n_best_parses[i][0], [], [], [])
                    if i == 0: highest_scoring_denotation = candidate_denotation
                    if candidate_denotation == d:
                        correct_denotation_parse = n_best_parses[i]
                        break
                    else:
                        print "candidate denotation " + str(
                            candidate_denotation) + " does not match known correct d " + str(d)
                if correct_denotation_parse is None:
                    print "WARNING: did not find parse with correct denotation " + str(d) + " for tokens " + str(x)
                elif highest_scoring_denotation != d:
                    T.append([x, highest_scoring_parse[0], highest_scoring_parse[1], highest_scoring_denotation,
                              correct_denotation_parse[0], correct_denotation_parse[1], d])
                else:
                    correct += 1
            if len(T) == 0:
                print "WARNING: training converged; " + str(correct) + "/" + str(len(D)) + " correct denotations"
                break
            random.shuffle(T)
            self.learner.learnFromDenotations(T, epochs)

    # read in data set of form utterance\nsemantic_form\n\n...
    def read_in_paired_utterance_semantics(self, fname):
        D = []
        f = open(fname, 'r')
        f_lines = f.readlines()
        f.close()
        i = 0
        while i < len(f_lines):
            tokens = self.tokenize(f_lines[i].strip())
            form = self.lexicon.read_semantic_form_from_str(f_lines[i + 1].strip(), None, None, [])
            D.append([tokens, form])
            i += 3
        return D

    # take in a data set D=(x,y) for x tokens and y correct semantic form and update the learner's parameters
    def train_learner_on_semantic_forms(self, D, reset_learner=False, epochs=10, k=None, n=10):
        if k is None: k = self.beam_width
        for e in range(0, epochs):
            print "epoch " + str(e)  # DEBUG
            T = []
            all_matches = True
            for [x, y] in D:
                print x  # DEBUG
                n_best = self.parse_tokens(x, k, n)
                highest_scoring_parse = n_best[0]
                for i in range(1, len(n_best)):
                    if not y.__eq__(n_best[i][0]) and n_best[i][2] + 1 >= n_best[i - 1][2]:
                        highest_scoring_parse = n_best[i]
                        break
                T.append([x, [highest_scoring_parse[0]],
                          [y]])  # nest parses because feature extractor expects partials (list of partial parse trees)
                if not y.__eq__(highest_scoring_parse[0]):
                    print "mismatch: " + self.print_parse(y) + " != " + self.print_parse(
                        highest_scoring_parse[0])  # DEBUG
                    print self.print_semantic_parse_result(highest_scoring_parse[1])  # DEBUG
                    all_matches = False
                else:
                    "WARNING: could not find valid parse for '" + " ".join(x) + "' during training; ignoring example"
            if all_matches == True:
                print "WARNING: training converged at epoch " + str(e)
                break
            random.shuffle(T)
            self.learner.learnFromSemanticForms(T, reset_learner, epochs)

    # iterate over partials and add genlex entries to lexicon
    def add_genlex_entries_to_lexicon_from_partial(self, partial_bracketing):
        for pb in partial_bracketing:
            self.add_genlex_entries_to_lexicon_from_bracketing(pb)

    # trace a parse for genlex entries and make them mapped members of the lexicon
    def add_genlex_entries_to_lexicon_from_bracketing(self, bracketing):
        if type(bracketing) is tuple:
            return
        if type(bracketing[0]) is int:
            candidate = True
            for i in range(0, 2):
                if type(bracketing[1][i]) is not tuple:
                    candidate = False
                elif bracketing[1][i][0] is not None:
                    candidate = False
                elif type(bracketing[1][i][1]) is not str:
                    candidate = False
                if not candidate:
                    break
            if candidate and bracketing[1][0][1] == bracketing[1][1][1]:
                sur = bracketing[1][0][1]
                if sur not in self.lexicon.surface_forms:
                    self.lexicon.surface_forms.append(sur)
                    self.lexicon.entries.append([bracketing[0]])
                else:
                    sur_idx = self.lexicon.surface_forms.index(sur)
                    if bracketing[0] not in self.lexicon.entries[sur_idx]:
                        self.lexicon.entries[sur_idx].append(bracketing[0])
                print "added lexical entry '"+sur+"' : "+self.print_parse(self.lexicon.semantic_forms[bracketing[0]]) #  DEBUG
                print "surface forms: "+str(self.lexicon.surface_forms)
                print "entries: "+str(self.lexicon.entries)
        if type(bracketing[1]) is list:
            for i in range(0, len(bracketing[1])):
                self.add_genlex_entries_to_lexicon_from_bracketing(bracketing[1][i])

    # tokenizes str and passes it to a function to parse tokens
    def parse_expression(self, s, k=None, n=1):
        tokens = self.tokenize(s)
        return self.parse_tokens(tokens, k=k, n=n)

    # given tokens, returns list of up to k [parse,semantic parse tree,score] pairs;
    # less than k returned if fewer than k full parses could be found
    def parse_tokens(self, tokens, k=None, n=1):
        if k is None: k = self.beam_width
        self.new_adjectives = []

        candidate_semantic_forms = {}
        for i in range(0, len(tokens)):
            for j in range(i, len(tokens)):
                forms = self.lexicon.get_semantic_forms_for_surface_form(" ".join(tokens[i:j+1]))
                if len(forms) == 0 and i == j: # add missing tokens to surface forms, but not spans of tokens
                    if tokens[i] not in self.lexicon.surface_forms:
                        self.lexicon.surface_forms.append(tokens[i])
                        self.lexicon.entries.append([])
                candidate_semantic_forms[(i, j)] = forms
        # print "tokens "+str(tokens)  # DEBUG
        # print "candidate forms "+str(candidate_semantic_forms)  # DEBUG

        # incrementally allow more None assignments, initially trying for none
        null_assignments_allowed = 0
        full_parses = []  # contains both truly full parses and lists of trees such that all but one are leaves
        while len(full_parses) < n and null_assignments_allowed < len(tokens):

            # initialize set of partial parses to each token assigned each possible meaning
            # a parse is represented as a vector of trees whose leaves in order are the tokens to mark
            # a complete parse is a single tree
            # parse tree leaves are lexicon semantic form idxs; upper levels are instantiated SemanticNodes
            # tokens assigned to None meaning are pushed in order to the back of the list for equality testing later
            partial_semantic_parse_trees = self.expand_partial_parse_with_additional_tokens(
                                            [], tokens, candidate_semantic_forms, null_assignments_allowed, 0)
            partial_parse_forms = [[pspt[i][0] for i in range(0, len(pspt))] for pspt in partial_semantic_parse_trees]
            partial_parses = [[partial_parse_forms[i], partial_semantic_parse_trees[i]] for i in
                              range(0, len(partial_parse_forms))]
            # print "Nones allowed:\t"+str(null_assignments_allowed)  # DEBUG
            # print "partial_semantic_parse_trees: "+str(partial_semantic_parse_trees)  # DEBUG
            # print "partial_parse_forms: "+str(partial_parse_forms)  # DEBUG
            # print "partial_parses: "+str(partial_parses)  # DEBUG

            # until full parses reaches some threshold k, score all partials and choose one to expand,
            # adding full parses to the set; don't allow expansions leading to bad partials
            # if a partial parse cannot be further expanded into a full parse, add it to the bad partial parses list
            # and remove it from partial
            bad_partial_parses = []
            scores = [self.learner.scoreParse(tokens, pp) for pp in partial_parses]
            while len(full_parses) < k and len(partial_parses) > 0:
                # print "partial parses:\t"+str(len(partial_parses))  # DEBUG
                # print "bad parses:\t"+str(len(bad_partial_parses))  # DEBUG
                # print "full parses:\t"+str(len(full_parses))  # DEBUG
                # choose maximum scoring parse and try to expand it by examining all possible leaf connections
                max_score_idx = scores.index(max(scores))
                if (null_assignments_allowed == len(
                        tokens) - 1):  # no FA to be done, so just add highest scoring parse to full parses
                    pp = [None if p is None else self.lexicon.semantic_forms[p] for p in
                          partial_parses[max_score_idx][0]]
                    full_parses.append([pp, partial_parses[max_score_idx][1]])
                else:
                    #print "expanding: "  # DEBUG
                    #for p in partial_parses[max_score_idx][0]:  # DEBUG
                    #    print self.print_parse(self.lexicon.semantic_forms[p] if type(p) is int else p, True)
                    #print self.print_semantic_parse_result(partial_parses[max_score_idx][1])  # DEBUG
                    new_partials = [[cpp, cpsp] for [cpp, cpsp] in
                                self.expand_partial_parse(partial_parses[max_score_idx]) if
                                cpp not in bad_partial_parses]
                    if len(new_partials) == 0:  # then this parse is a bad partial
                        bad_partial_parses.append(partial_parses[max_score_idx])
                    # score new parses and add to partials / full where appropriate
                    new_partials_scores = [self.learner.scoreParse(tokens, npp) for npp in new_partials]
                    new_partials_beam_size = 0
                    while new_partials_beam_size < k and len(new_partials) > 0:
                        max_new_score_idx = new_partials_scores.index(max(new_partials_scores))
                        num_internal_trees = sum([1 if p is not None else 0 for p
                                                  in new_partials[max_new_score_idx][0]])
                        if num_internal_trees == 1:
                            if new_partials[max_new_score_idx] not in full_parses:
                                full_parses.append(new_partials[max_new_score_idx])
                            new_partials_beam_size += 1
                        elif new_partials[max_new_score_idx] not in partial_parses:
                            partial_parses.append(new_partials[max_new_score_idx])
                            scores.append(new_partials_scores[max_new_score_idx])
                            new_partials_beam_size += 1
                        del new_partials[max_new_score_idx]
                        del new_partials_scores[max_new_score_idx]
                # remove from partials list so we don't waste time expanding again
                del partial_parses[max_score_idx]
                del scores[max_score_idx]

            null_assignments_allowed += 1

        # score full parses and return n best in order with scores
        full_parse_scores = [self.learner.scoreParse(tokens, fp) for fp in full_parses]
        k_best = []
        while len(k_best) < n and len(full_parses) > 0:
            max_score_idx = full_parse_scores.index(max(full_parse_scores))
            internal_tree_flags = [1 if p is not None else 0 for p in full_parses[max_score_idx][0]]
            if 1 in internal_tree_flags:
                internal_tree_idx = internal_tree_flags.index(1)
                k_best.append([full_parses[max_score_idx][0][internal_tree_idx], full_parses[max_score_idx][1],
                               full_parse_scores[max_score_idx]])
            del full_parses[max_score_idx]
            del full_parse_scores[max_score_idx]
        return k_best

    # given partial parse
    # expand via function application (forwards and backwards), type-raising, and merge operations
    # attempt to escape un-parsable situations through `parsing with unknowns'
    def expand_partial_parse(self, p):
        partial = p[0]
        tree = p[1]
        possible_joins = []
        pairwise_joins = False
        for i in range(0, len(partial)):
            if partial[i] is None: continue
            lhs = rhs = None
            for lhs in range(i - 1, -1, -1):
                if partial[lhs] is not None: break
            for rhs in range(i + 1, len(partial)):
                if partial[rhs] is not None: break
            for j in [lhs, rhs]:
                if j <= -1 or j >= len(partial) or i == j:
                    continue  # edge cases

                refs = [self.lexicon.semantic_forms[partial[idx]] if type(partial[idx]) is int else partial[idx] for idx
                        in [i, j]]
                # try FA from i to j
                if self.can_perform_fa(i, j, refs[0], refs[1]):
                    objs = [copy.deepcopy(self.lexicon.semantic_forms[partial[idx]]) if type(partial[idx]) is int else
                            partial[idx] for idx in [i, j]]
                    joined_subtrees = self.perform_fa(objs[0], objs[1])
                    possible_joins.append([i, j, joined_subtrees, tree[i], tree[j], None])
                    pairwise_joins = True
                # try merge from i to j
                if self.can_perform_merge(refs[0], refs[1]):
                    merged_subtrees = self.perform_merge(refs[0], refs[1])
                    possible_joins.append([i, j, merged_subtrees, tree[i], tree[j], None])
                    pairwise_joins = True
            # try Type Raising on i
            ref = self.lexicon.semantic_forms[partial[i]] if type(partial[i]) is int else partial[i]
            if self.can_perform_type_raising(ref) == "DESC->N/N":
                raised_subtrees = self.perform_adjectival_type_raise(ref)
                possible_joins.append([i, i, raised_subtrees, tree[i], tree[i], None])

        # try parsing with unknowns if no pairwise connections remain
        if not pairwise_joins:
            for i in range(0, len(partial)):
                if partial[i] is not None: continue
                # proceed only if no meanings for token known (should be relaxed eventually; beam can handle exp)
                if (len(self.lexicon.entries[self.lexicon.surface_forms.index(tree[i][1])]) == 0
                        or tree[i][1] in self.new_adjectives):
                    for d in [-1, 1]:
                        candidate_assignments = self.genlex_for_missing_entry(partial, i, d, tree[i][1])
                        for candidate_assignment, syn_idx in candidate_assignments:
                            possible_joins.append([i, i, candidate_assignment, tree[i], tree[i], syn_idx])

        expanded_partials = []
        for join in possible_joins:
            f = True if join[0] < join[1] else False
            ordered_break = [join[0], join[1]] if f else [join[1], join[0]]
            expanded_partial = partial[:ordered_break[0]]
            expanded_tree = tree[:ordered_break[0]]
            expanded_partial.append(join[2])
            expanded_tree.append([join[5], [join[3], join[4]]])
            expanded_partial.extend(partial[ordered_break[0] + 1:ordered_break[1]])
            expanded_tree.extend(tree[ordered_break[0] + 1:ordered_break[1]])
            expanded_partial.extend(partial[ordered_break[1] + 1:])
            expanded_tree.extend(tree[ordered_break[1] + 1:])
            expanded_partials.append([expanded_partial, expanded_tree])
        return expanded_partials

    # return set of candidate entries for idx of given partial with respect to its neighbor in direction d
    # performs exact syntax/semantics match against lexicon; can only learn perfect synonyms
    # has special allowances for UNK assignments to tokens for certain syntactic categories
    def genlex_for_missing_entry(self, partial, idx, d, token):

        # ensure the given parameters are candidates for generating new lexical entries
        if partial[idx] is not None: return []
        i = idx+d
        if i < 0 or i == len(partial): return []
        if partial[i] is None: return []  # TODO: allow multi-word entries through sequence of None
        neighbor = self.lexicon.semantic_forms[partial[i]] if type(partial[i]) is int else partial[i]
        n_cat = self.lexicon.categories[neighbor.category]
        candidates = []
        adj_cats = []
        try:
            adj_cats.append(self.lexicon.categories.index('DESC'))
        except ValueError:
            pass
        try:
            adj_cats.append(self.lexicon.categories.index([self.lexicon.categories.index('N'), 1,
                                                           self.lexicon.categories.index('N')]))
        except ValueError:
            pass
        possible_adjective = False

        # try consuming n_cat in direction d
        consume_d = 0 if d == -1 else 1
        for form_idx in range(0, len(self.lexicon.semantic_forms)):
            form = self.lexicon.semantic_forms[form_idx]
            form_cat = self.lexicon.categories[form.category]
            if type(form_cat) is list and form_cat[1] == consume_d and form_cat[2] == neighbor.category:
                if form_cat in adj_cats:
                    possible_adjective = True
                else:
                    candidates.append([form, form_idx])

        # try being consumed by n_cat from direction d
        if (type(n_cat) is list
                and ((i < idx and n_cat[1] == 1) or (i > idx and n_cat[1] == 0))):
            # if neighbor is looking for an N, treat as UNK due to ambiguity
            if n_cat[2] == self.lexicon.categories.index('N'):
                # 'N' is a candidate to receive UNK token
                unk = SemanticNode.SemanticNode(None, None, None, False, idx=self.ontology.preds.index('UNK_E'))
                unk.type = self.ontology.types.index('e')
                unk.category = self.lexicon.categories.index('N')
                unk.set_return_type(self.ontology)
                candidates.append([unk, None])
            # if neighbor is looking for an adjective, stave off decision for later
            elif n_cat[2] in adj_cats:
                possible_adjective = True
            # look generally for consumable candidates
            else:
                for form_idx in range(0, len(self.lexicon.semantic_forms)):
                    form = self.lexicon.semantic_forms[form_idx]
                    if n_cat[2] == form.category:
                        candidates.append([form, form_idx])

        # if procedure has determined that some candidate is an adjective, add as new entry to the
        # ontology of predicates, since perception should eventually merge/split these appropriately
        # liberally add new adjective predicate SemanticNode to the lexicon and a surface->semantic entry
        # TODO: separate KB and perceptual predicates, only fire this procedure to prevent synonym perceptuals
        if possible_adjective:
            if token not in self.new_adjectives:
                self.ontology.preds.append(token)
                self.ontology.entries.append(self.ontology.read_type_from_str("<e,t>"))
                print self.ontology.preds  # DEBUG
                print self.ontology.entries  # DEBUG
                self.ontology.num_args.append(1)
                new_lex = [token + " :- DESC : " + token]
                print "adding adjective " + str(new_lex)  # DEBUG
                self.lexicon.expand_lex_from_strs(new_lex, self.lexicon.surface_forms,
                                                  self.lexicon.semantic_forms, self.lexicon.entries,
                                                  self.lexicon.pred_to_surface)
                print "surface forms: "+str(self.lexicon.surface_forms)  # DEBUG
                print "entries: "+str(self.lexicon.entries)  # DEBUG
                self.new_adjectives.append(token)
            adj_idx = self.lexicon.entries[self.lexicon.surface_forms.index(token)][0]
            candidates.append([self.lexicon.semantic_forms[adj_idx], adj_idx])

        return candidates

    # return DESC : pred raised to N/N : lambda x.(pred(x))
    def perform_adjectival_type_raise(self, A):
        # print "performing adjectival raise with '"+self.print_parse(A,True)+"'" #DEBUG
        child_pred = copy.deepcopy(A)
        child_pred.children = [
            SemanticNode.SemanticNode(child_pred, self.ontology.types.index('e'), self.lexicon.categories.index('N'),
                                      is_lambda=True, lambda_name=1, is_lambda_instantiation=False)]
        child_pred.category = self.lexicon.categories.index('N')  # change from DESC return type
        raised = SemanticNode.SemanticNode(None, None, None, True, lambda_name=1, is_lambda_instantiation=True)
        raised.type = self.ontology.types.index('e')
        raised.category = self.lexicon.categories.index(
            [self.lexicon.categories.index('N'), 1, self.lexicon.categories.index('N')])
        raised.children = [child_pred]
        raised.set_return_type(self.ontology)
        # print "performed adjectival raise with '"+self.print_parse(A,True)+"' to form '"+self.print_parse(raised,True)+"'" #DEBUG
        return raised

    # return true if A is a candidate for type-raising
    def can_perform_type_raising(self, A):
        if 'DESC' not in self.lexicon.categories: return False  # type-raising not allowed by lexicon writer
        if A is None: return False
        # check for DESC : pred -> N/N : lambda x.(pred(x)) raise
        if A.children is None and A.category == self.lexicon.categories.index('DESC'):
            return "DESC->N/N"
        return False

    # return A<>B; A and B must have matching lambda headers and syntactic categories to be AND merged
    def perform_merge(self, A, B):
        # print "performing Merge with '"+self.print_parse(A,True)+"' taking '"+self.print_parse(B,True)+"'" #DEBUG

        and_idx = self.ontology.preds.index('and')

        if A.is_lambda_instantiation:
            A_B_merged = copy.deepcopy(A)
            A_B_merged.category = A.category
            innermost_outer_lambda = A_B_merged
            A_child = A.children[0]
            B_child = B.children[0]
            while innermost_outer_lambda.children != None and innermost_outer_lambda.children[0].is_lambda and innermost_outer_lambda.children[0].is_lambda_instantiation:
                innermost_outer_lambda = innermost_outer_lambda.children[0]
                A_child = A.children[0]
                B_child = B.children[0]
            innermost_outer_lambda.children = [
                SemanticNode.SemanticNode(innermost_outer_lambda, self.ontology.entries[and_idx],
                                          innermost_outer_lambda.children[0].category, False, idx=and_idx)]
            innermost_outer_lambda.children[0].children = [copy.deepcopy(A_child), copy.deepcopy(B_child)]

            # 'and' adopts type taking each child's and returning the same
            A_child.set_return_type(self.ontology)
            B_child.set_return_type(self.ontology)
            input_type = [A_child.return_type, A_child.return_type]
            if input_type not in self.ontology.types:
                self.ontology.types.append(input_type)
            full_type = [A_child.return_type, self.ontology.types.index(input_type)]
            if full_type not in self.ontology.types:
                self.ontology.types.append(full_type)
            innermost_outer_lambda.children[0].type = self.ontology.types.index(full_type)
            innermost_outer_lambda.children[0].set_return_type(self.ontology)
        else:
            input_type = [A.return_type, A.return_type]
            if input_type not in self.ontology.types:
                self.ontology.types.append(input_type)
            full_type = [A.return_type, self.ontology.types.index(input_type)]
            if full_type not in self.ontology.types:
                self.ontology.types.append(full_type)
            A_B_merged = SemanticNode.SemanticNode(None, self.ontology.types.index(full_type),
                                                   A.category, False, idx=and_idx)
            A_B_merged.children = [A, B]
            A_B_merged.set_return_type(self.ontology)

        # print "performed Merge with '"+self.print_parse(A,True)+"' taking '"+self.print_parse(B,True)+"' to form '"+self.print_parse(A_B_merged,True)+"'" #DEBUG
        return A_B_merged

    # return true if A,B can be merged
    def can_perform_merge(self, A, B):
        if A is None or B is None: return False
        # if not A.is_lambda or not A.is_lambda_instantiation or not B.is_lambda or not B.is_lambda_instantiation:
        #     return False
        # if type(self.lexicon.categories[A.category]) is not list or type(
        #         self.lexicon.categories[B.category]) is not list:
        #     return False
        if self.lexicon.categories[A.category] != self.lexicon.categories[B.category]:
            return False
        if A.return_type is None: A.set_return_type(self.ontology)
        if B.return_type is None: B.set_return_type(self.ontology)
        if A.return_type != B.return_type: return False
        curr_A = A
        curr_B = B
        while curr_A.is_lambda and curr_A.is_lambda_instantiation:
            if curr_A.return_type is None: curr_A.set_return_type(self.ontology)
            if curr_B.return_type is None: curr_B.set_return_type(self.ontology)
            if (not curr_B.is_lambda or not curr_B.is_lambda_instantiation or curr_B.type != curr_A.type
                    or curr_A.return_type != curr_B.return_type):
                return False
            curr_A = curr_A.children[0]
            curr_B = curr_B.children[0]
        return True

    # return A(B); A must be lambda headed with type equal to B's root type
    def perform_fa(self, A, B):
        # print "performing FA with '"+self.print_parse(A,True)+"' taking '"+self.print_parse(B,True)+"'" #DEBUG

        # if A is 'and', apply B to children
        if not A.is_lambda and self.ontology.preds[A.idx] == 'and':
            for i in range(0, len(A.children)):
                c = A.children[i]
                c_obj = copy.deepcopy(self.lexicon.semantic_forms[c]) if type(c) is int else c
                A.children[i] = self.perform_fa(c_obj, B)
            return A

        # else, proceed as expected
        A_FA_B = copy.deepcopy(
            A.children[0])  # A is lambda headed and so has a single child which will be the root of the composed tree
        A_FA_B.parent = None
        # traverse A_FA_B and replace references to lambda_A with B
        A_FA_B_deepest_lambda = A.lambda_name
        to_traverse = [[A_FA_B, A_FA_B_deepest_lambda]]
        while len(to_traverse) > 0:
            [curr, deepest_lambda] = to_traverse.pop()
            entire_replacement = False
            if curr.is_lambda and curr.is_lambda_instantiation:
                deepest_lambda = curr.lambda_name
            elif curr.is_lambda and not curr.is_lambda_instantiation and curr.lambda_name == A.lambda_name:  # an instance of lambda_A to be replaced by B
                # print "substituting '"+self.print_parse(B,True)+"' for '"+self.print_parse(curr,True)+"' with lambda offset "+str(deepest_lambda) #DEBUG
                if (not B.is_lambda and self.ontology.preds[B.idx] == 'and'
                        and curr.children is not None and B.children is not None):
                    # if B is 'and', can preserve it and interleave A's children as arguments
                    for i in range(0, len(B.children)):
                        c = B.children[i]
                        c_obj = copy.deepcopy(self.lexicon.semantic_forms[c]) if type(c) is int else c
                        if self.can_perform_type_raising(c_obj) == "DESC->N/N":
                            c_obj = self.perform_adjectival_type_raise(c_obj)
                        B.children[i] = self.perform_fa(c_obj, curr.children[0])
                        B.children[i].set_return_type(self.ontology)
                    curr.copy_attributes(B)
                    curr.type = self.ontology.types.index([curr.children[0].return_type, self.ontology.types.index(
                        [curr.children[0].return_type, curr.children[0].return_type])])
                elif curr.parent is None:
                    if curr.children is None:
                        # print "...whole tree is instance" #DEBUG
                        curr.copy_attributes(B)  # instance is whole tree; add nothing more and loop will now exit
                    elif B.children is None:
                        # print "...instance heads tree; preserve children taking B"
                        curr.copy_attributes(B, deepest_lambda, preserve_children=True)
                    else:
                        sys.exit("Error: incompatible parentless, childed node A with childed node B")
                    entire_replacement = True
                    curr.category = self.lexicon.categories[A.category][0]  # take on return type of A
                else:
                    for curr_parent_matching_idx in range(0, len(curr.parent.children)):
                        if curr.parent.children[curr_parent_matching_idx] == curr: break
                    if curr.children is None:
                        # print "...instance of B will preserve its children" #DEBUG
                        curr.parent.children[curr_parent_matching_idx].copy_attributes(B, deepest_lambda,
                                                                                      preserve_parent=True)  # lambda instance is a leaf
                    else:
                        if B.children is None:
                            # print "...instance of B will keep children from A" #DEBUG
                            curr.parent.children[curr_parent_matching_idx].copy_attributes(B, deepest_lambda,
                                                                                          preserve_parent=True,
                                                                                          preserve_children=True)
                        else:
                            # print "...instance of A and B have matching lambda headers to be merged" #DEBUG
                            B_without_lambda_headers = B
                            num_leading_lambdas = 0
                            while B_without_lambda_headers.is_lambda and B_without_lambda_headers.is_lambda_instantiation:
                                B_without_lambda_headers = B_without_lambda_headers.children[0]
                                num_leading_lambdas += 1
                            curr.parent.children[curr_parent_matching_idx].copy_attributes(B_without_lambda_headers,
                                                                                          num_leading_lambdas,
                                                                                          preserve_parent=True,
                                                                                          preserve_children=False)
                    curr.parent.children[curr_parent_matching_idx].set_return_type(self.ontology)  # not sure we need this
            if not entire_replacement and curr.children is not None: to_traverse.extend([[c, deepest_lambda] for c in curr.children])
        self.renumerate_lambdas(A_FA_B, [])
        A_FA_B.category = self.lexicon.categories[A.category][0]
        # print "performed FA with '"+self.print_parse(A,True)+"' taking '"+self.print_parse(B,True)+"' to form '"+self.print_parse(A_FA_B,True)+"'" #DEBUG
        return A_FA_B

    # return true if A(B) is a valid for functional application
    def can_perform_fa(self, i, j, A, B):
        if A is None or B is None: return False
        if A.category is None or type(self.lexicon.categories[A.category]) is not list or (
                            i - j > 0 and self.lexicon.categories[A.category][1] == 1) or (
                            i - j < 0 and self.lexicon.categories[A.category][1] == 0):
            return False  # B is left/right when A expects right/left
        if not A.is_lambda or not A.is_lambda_instantiation or A.type != B.return_type:
            return False
        if self.lexicon.categories[A.category][2] != B.category:
            return False  # B is not the input category A expects
        if A.parent is None and A.is_lambda and not A.is_lambda_instantiation:
            return True  # the whole tree of A will be replaced with the whole tree of B
        to_traverse = [B]
        B_lambda_context = []
        while len(to_traverse) > 0:
            curr = to_traverse.pop()
            if curr.is_lambda and curr.is_lambda_instantiation:
                B_lambda_context.append(curr.type)
            else:
                break
            to_traverse.extend(B.children)
        return self.lambda_value_replacements_valid(A.children[0], A.lambda_name, [], B,
                                                 B_lambda_context)  # return True if all instances of A lambda appear in lambda contexts identical to what B expects

    def lambda_value_replacements_valid(self, A, lambda_name, A_lambda_context, B, B_lambda_context):
        # print "checking whether '"+self.printParse(A)+"' lambda "+str(lambda_name)+" instances can be replaced by '"+self.printParse(B)+"' under contexts "+str(A_lambda_context)+","+str(B_lambda_context) #DEBUG
        if A.is_lambda and A.is_lambda_instantiation:
            extended_context = A_lambda_context[:]
            extended_context.append(A.type)
            return self.lambda_value_replacements_valid(A.children[0], lambda_name, extended_context, B, B_lambda_context)
        if A.is_lambda and not A.is_lambda_instantiation and A.lambda_name == lambda_name:  # this is an instance of A's lambda to be replaced by B
            if A.children is not None and B.children is not None:  # the instance takes arguments
                if len(A_lambda_context) != len(B_lambda_context): return False  # the lambda contexts differ in length
                matches = [1 if A_lambda_context[i] == B_lambda_context[i] else 0 for i in
                           range(0, len(A_lambda_context))]
                if sum(matches) != len(A_lambda_context): return False  # the lambda contexts differ in content
                return True
            return True  # ie A, B have no children
        if A.children is None: return True
        valid_through_children = True
        for c in A.children:
            valid_through_children = self.lambda_value_replacements_valid(c, lambda_name, A_lambda_context, B,
                                                                       B_lambda_context)
            if not valid_through_children: break
        return valid_through_children

    def renumerate_lambdas(self, A, lambdas):
        if A.is_lambda:
            if A.is_lambda_instantiation:
                lambdas.append(A.lambda_name)
                A.lambda_name = len(lambdas)
            else:
                A.lambda_name = lambdas.index(A.lambda_name) + 1
        if A.children is not None:
            for c in A.children:
                self.renumerate_lambdas(c, lambdas[:])

    # recursive procedure to build all possible leaf sets as partial parses
    def expand_partial_parse_with_additional_tokens(self, partial, tokens, candidate_semantic_forms, nulls_remaining, idx):
        expanded_partials = []
        span = 0
        # print partial, tokens[idx] #DEBUG
        # r = raw_input()
        while (idx, idx+span) in candidate_semantic_forms:
            none_valid = -1 if span == 0 else 0
            for s in range(none_valid, len(candidate_semantic_forms[idx, idx+span])):
                expanded = partial[:]
                if s == -1 and nulls_remaining > 0:
                    expanded.append((None, tokens[idx]))
                elif s == -1:
                    continue
                else:
                    expanded.append((candidate_semantic_forms[idx, idx+span][s], " ".join(tokens[idx:idx+span+1])))
                if expanded[-1][0] is None or nulls_remaining < len(tokens)-idx:
                    if len(tokens)-idx > span+1:
                        dx = -1 if expanded[-1][0] is None else 0
                        next_exp = self.expand_partial_parse_with_additional_tokens(
                            expanded, tokens, candidate_semantic_forms, nulls_remaining+dx, idx+span+1)
                        expanded_partials.extend(next_exp)
                    else:
                        expanded_partials.append(expanded)
            span += 1
        return expanded_partials

    # turn a string into a sequence of tokens to be assigned semantic meanings
    def tokenize(self, s):
        s = s.replace("'s", " 's")
        str_parts = s.split()
        for i in range(0, len(str_parts)):
            if str_parts[i] == "an": str_parts[i] = "a"
        return [p for p in str_parts if len(p) > 0]
