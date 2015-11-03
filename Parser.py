__author__ = 'jesse'

import sys
import copy
import random
import math
import SemanticNode


class Parser:
    def __init__(self, ont, lex, learner, grounder, beam_width=10, safety=False):
        self.ontology = ont
        self.lexicon = lex
        self.learner = learner
        self.grounder = grounder
        self.beam_width = beam_width
        self.new_adjectives = []  # cleared before every parsing job
        self.safety = safety
        self.commutative_idxs = [self.ontology.preds.index('and'), self.ontology.preds.index('or')]

    # print a SemanticNode as a string using the known ontology
    def print_parse(self, p, show_category=False, show_non_lambda_types=False):
        if p is None: return "NONE"
        elif show_category and p.category is not None:
            s = self.lexicon.compose_str_from_category(p.category) + " : "
        else:
            s = ''
        t = self.ontology.compose_str_from_type(p.type)
        if p.is_lambda:
            if p.is_lambda_instantiation:
                s += "lambda " + str(p.lambda_name) + ":" + t + "."
            else:
                s += str(p.lambda_name)
        else:
            s += self.ontology.preds[p.idx]
            if show_non_lambda_types:
                s += ":"+t
        if p.children is not None:
            s += '(' + ','.join([self.print_parse(c, show_non_lambda_types=show_non_lambda_types)
                                 for c in p.children]) + ')'
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

    def tokens_of_parse_tree(self, sp, allow_None=False, collapse_adj=False, disallow_None_adj=False):
        if collapse_adj and type(sp) is list and type(sp[1]) is list and sp[1][0] == sp[1][1]:
            return self.tokens_of_parse_tree(sp[1][0], allow_None=allow_None, collapse_adj=collapse_adj)
        elif type(sp[1]) is str:
            if type(sp[0]) is int:
                return sp[1]
            elif allow_None and sp[0] is None:
                # and len(self.lexicon.entries[self.lexicon.surface_forms.index(sp[1])]) == 0):
                # the second part of this conditional means we can't learn any new meanings for existing words
                is_adj = False
                if disallow_None_adj:
                    for sem_idx in self.lexicon.entries[self.lexicon.surface_forms.index(sp[1])]:
                        if self.lexicon.semantic_forms[sem_idx].category == self.lexicon.categories.index('DESC'):
                            is_adj = True
                if not is_adj or not disallow_None_adj:
                    return None
        elif type(sp[1]) is list:
            return [self.tokens_of_parse_tree(
              m, allow_None=allow_None, collapse_adj=collapse_adj, disallow_None_adj=disallow_None_adj) for m in sp[1]]
        else:
            sys.exit("Unhandled input in tokens_of_parse_tree: "+str(sp))
        return False

    def flatten(self, l):
        if type(l) is not list:
            return [l]
        r = []
        for li in l:
            r.extend(self.flatten(li))
        return r

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
                    # print "WARNING: could not find valid parse for tokens"+str(x)
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
            self.learner.learn_from_denotations(T, epochs)

    # read in data set of form utterance\nsemantic_form\n\n...
    def read_in_paired_utterance_semantics(self, fname, allow_expanding_ont=False):
        D = []
        f = open(fname, 'r')
        f_lines = f.readlines()
        f.close()
        i = 0
        while i < len(f_lines):
            if len(f_lines[i].strip()) == 0:
                i += 1
                continue
            tokens = self.tokenize(f_lines[i].strip())
            form = self.lexicon.read_semantic_form_from_str(
                f_lines[i + 1].strip(), None, None, [], allow_expanding_ont=allow_expanding_ont)
            D.append([tokens, form])
            i += 3
        return D

    # take in a data set D=(x,y) for x tokens and y correct semantic form and update the learner's parameters
    def train_learner_on_semantic_forms(
            self, D, epochs=10, k=None, n=3, allow_UNK_E=True, generator=None):
        if k is None:
            k = self.beam_width
        for e in range(0, epochs):
            print "epoch " + str(e)  # DEBUG
            T = []
            num_trainable = 0
            num_matches = 0
            num_fails = 0
            for [x, y] in D:
                generator_genlex = None if generator is None else [generator, x, y]
                # print "training on pair "+str(x)+", "+self.print_parse(y, True)  # DEBUG
                if generator is not None:
                    generator.flush_seen_nodes()
                chosen_parse = None
                correct_parse = None
                any_parse = False
                match = False
                none_max = 0
                num_examined = 0
                while none_max <= math.ceil(math.sqrt(len(x))) and correct_parse is None and n-num_examined > 0:
                    n_best = self.parse_tokens(
                        x, k, n-num_examined, none_range=[none_max, none_max+1],
                        allow_UNK_E=allow_UNK_E, generator_genlex=generator_genlex)
                    if len(n_best) > 0:
                        any_parse = True
                        if chosen_parse is None:
                            chosen_parse = n_best[0]
                        for i in range(0, len(n_best)):
                            num_examined += 1
                            if y.equal_allowing_commutativity(n_best[i][0],
                                                              self.commutative_idxs, ontology=self.ontology):
                                correct_parse = n_best[i]
                                if i == 0 and chosen_parse[0].equal_allowing_commutativity(
                                        n_best[i][0], self.commutative_idxs):
                                    match = True
                                    num_matches += 1
                                else:
                                    num_trainable += 1
                                break
                    none_max += 1
                if correct_parse is None:
                    print "WARNING: could not find correct parse for input sequence "+str(x)
                    num_fails += 1
                    continue
                if not match:
                    T.append([x, chosen_parse, None,
                              x, correct_parse, None])
                if not any_parse:
                    print "WARNING: could not find valid parse for '" + " ".join(x) + "' during training"
                    num_fails += 1
            print "matched "+str(num_matches)+"/"+str(len(D))  # DEBUG
            print "trained "+str(num_trainable)+"/"+str(len(D))  # DEBUG
            print "failed "+str(num_fails)+"/"+str(len(D))  # DEBUG
            if num_trainable == 0:
                print "WARNING: training converged at epoch " + str(e)
                break
            print "training on "+str(len(T))+" examples"  # DEBUG
            random.shuffle(T)
            self.learner.learn_from_actions(T)

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
    def parse_expression(self, s, k=None, n=1, allow_UNK_E=True, generator_genlex=None):
        tokens = self.tokenize(s)
        return self.parse_tokens(tokens, k=k, n=n, allow_UNK_E=allow_UNK_E, generator_genlex=generator_genlex)

    # given tokens, returns list of up to n [parse, parse tree, parse trace, score]
    # less than n returned if fewer than n full parses could be found
    def parse_tokens(self, tokens, k=None, n=1, none_range=None, allow_UNK_E=True, generator_genlex=None):
        if k is None:
            k = self.beam_width
        if none_range is None:
            none_range = [0, len(tokens)]
        self.new_adjectives = []
        iterations_limit = max([1000, int(math.pow(2, len(tokens)))])

        candidate_semantic_forms = {}
        for i in range(0, len(tokens)):
            for j in range(i, len(tokens)):
                forms = self.lexicon.get_semantic_forms_for_surface_form(" ".join(tokens[i:j+1]))
                if len(forms) == 0 and i == j:  # add missing tokens to surface forms, but not spans of tokens
                    if tokens[i] not in self.lexicon.surface_forms:
                        self.lexicon.surface_forms.append(tokens[i])
                        self.lexicon.entries.append([])
                candidate_semantic_forms[(i, j)] = forms
        # print "tokens "+str(tokens)  # DEBUG
        # print "candidate forms "+str(candidate_semantic_forms)  # DEBUG

        # incrementally allow more None assignments, initially trying for none
        iterations = 0
        null_assignments_allowed = none_range[0]
        full_parses = []  # contains both truly full parses and lists of trees such that all but one are leaves
        while len(full_parses) < n and null_assignments_allowed < none_range[1]:

            # initialize set of partial parses to each token assigned each possible meaning
            # a parse is represented as a vector of trees whose leaves in order are the tokens to mark
            # a complete parse is a single tree
            # parse tree leaves are lexicon semantic form idxs; upper levels are instantiated SemanticNodes
            # tokens assigned to None meaning are pushed in order to the back of the list for equality testing later
            partial_semantic_parse_trees = self.expand_partial_parse_with_additional_tokens(
                                            [], tokens, candidate_semantic_forms, null_assignments_allowed, 0)
            partial_parse_forms = [[pspt[i][0] for i in range(0, len(pspt))] for pspt in partial_semantic_parse_trees]
            partial_parses = [[partial_parse_forms[i], partial_semantic_parse_trees[i], None] for i in
                              range(0, len(partial_parse_forms))]
            used_partials = []
            # print "Nones allowed:\t"+str(null_assignments_allowed)  # DEBUG
            # print "partial_semantic_parse_trees: "+str(partial_semantic_parse_trees)  # DEBUG
            # print "partial_parse_forms: "+str(partial_parse_forms)  # DEBUG
            # print "partial_parses: "+str(partial_parses)  # DEBUG

            # until full parses reaches some threshold n, score all partials and choose one to expand,
            # adding full parses to the set; don't allow expansions leading to bad partials
            # if a partial parse cannot be further expanded into a full parse, add it to the bad partial parses list
            # and remove it from partial
            bad_partial_parses = []
            scores = [self.learner.score_parse(tokens, [pp[0], pp[1], None]) for pp in partial_parses]  # score without history during parsing
            while (len(full_parses) < n and len(partial_parses) - len(used_partials) > 0 and
                    iterations < iterations_limit*(1+len(full_parses))):
                iterations += 1
                # print "partial parses:\t"+str(len(partial_parses)-len(used_partials))  # DEBUG
                # print "bad parses:\t"+str(len(bad_partial_parses))  # DEBUG
                # print "full parses:\t"+str(len(full_parses))  # DEBUG
                # choose maximum scoring parse and try to expand it by examining all possible leaf connections
                max_score_idx = scores.index(max(scores))
                parse_trace = [[partial_parses[max_score_idx][0],
                               partial_parses[max_score_idx][1]]]
                next_idx = partial_parses[max_score_idx][2]
                while next_idx is not None:
                    parse_trace.append([partial_parses[next_idx][0],
                                        partial_parses[next_idx][1]])
                    next_idx = partial_parses[next_idx][2]
                if (null_assignments_allowed == len(
                        tokens) - 1):  # no FA to be done, so just add highest scoring parse to full parses
                    pp = [None if p is None else self.lexicon.semantic_forms[p] for p in
                          partial_parses[max_score_idx][0]]
                    full_parses.append([pp, partial_parses[max_score_idx][1], parse_trace])
                else:
                    # print "expanding: "  # DEBUG
                    # print [self.print_parse(self.lexicon.semantic_forms[p] if type(p) is int else p, True)  # DEBUG
                    #        for p in partial_parses[max_score_idx][0]]  # DEBUG
                    # print self.print_semantic_parse_result(partial_parses[max_score_idx][1])  # DEBUG
                    new_partials = [[cpp, cpsp, max_score_idx] for [cpp, cpsp] in
                                self.expand_partial_parse(
                                    partial_parses[max_score_idx], allow_UNK_E=allow_UNK_E,
                                    generator_genlex=generator_genlex) if
                                cpp not in bad_partial_parses]
                    # print "num new partials: "+str(len(new_partials))  # DEBUG
                    if len(new_partials) == 0:  # then this parse is a bad partial
                        bad_partial_parses.append(partial_parses[max_score_idx])
                    # score new parses and add to partials / full where appropriate
                    new_partials_scores = [self.learner.score_parse(
                        tokens, [npp[0], npp[1], None]) for npp in new_partials]  # score without history during parsing
                    new_partials_beam_size = 0
                    while new_partials_beam_size < k and len(new_partials) > 0:
                        max_new_score_idx = new_partials_scores.index(max(new_partials_scores))
                        num_internal_trees = sum([1 if p is not None else 0 for p
                                                  in new_partials[max_new_score_idx][0]])
                        if num_internal_trees == 1:
                            if new_partials[max_new_score_idx] not in full_parses:
                                full_parses.append([new_partials[max_new_score_idx][0],
                                                    new_partials[max_new_score_idx][1],
                                                    parse_trace])
                            new_partials_beam_size += 1
                        elif new_partials[max_new_score_idx][0] not in [pp[0] for pp in partial_parses]:
                            partial_parses.append(new_partials[max_new_score_idx])
                            scores.append(new_partials_scores[max_new_score_idx])
                            new_partials_beam_size += 1
                        del new_partials[max_new_score_idx]
                        del new_partials_scores[max_new_score_idx]
                # remove from partials list so we don't waste time expanding again
                used_partials.append(max_score_idx)
                scores[max_score_idx] = -sys.maxint

            null_assignments_allowed += 1

        # if len(full_parses) < n:
        #     print "WARNING: exceeded parser iterations limit "+str(iterations_limit)+" for parse of "+\
        #         str(tokens)+" with only "+str(len(full_parses))+"/"+str(n)+" candidates"

        # score full parses and return n best in order with scores
        full_parse_scores = [self.learner.score_parse(tokens, fp) for fp in full_parses]  # uses trace history
        k_best = []
        while len(k_best) < n and len(full_parses) > 0:
            max_score_idx = full_parse_scores.index(max(full_parse_scores))
            internal_tree_flags = [1 if p is not None else 0 for p in full_parses[max_score_idx][0]]
            if 1 in internal_tree_flags:
                internal_tree_idx = internal_tree_flags.index(1)
                k_best.append([full_parses[max_score_idx][0][internal_tree_idx], full_parses[max_score_idx][1],
                               full_parses[max_score_idx][2], full_parse_scores[max_score_idx]])
            del full_parses[max_score_idx]
            del full_parse_scores[max_score_idx]
        return k_best

    # given partial parse
    # expand via function application (forwards and backwards), type-raising, and merge operations
    # attempt to escape un-parsable situations through `parsing with unknowns'
    def expand_partial_parse(self, p, allow_UNK_E, generator_genlex):
        generator = None if generator_genlex is None else generator_genlex[0]
        known_tokens = None if generator_genlex is None else generator_genlex[1]
        known_root = None if generator_genlex is None else generator_genlex[2]
        partial = p[0]
        tree = p[1]
        possible_joins = []
        pairwise_joins = False
        for i in range(0, len(partial)):
            if partial[i] is None:
                continue
            lhs = rhs = None
            for lhs in range(i - 1, -1, -1):
                if partial[lhs] is not None:
                    break
            for rhs in range(i + 1, len(partial)):
                if partial[rhs] is not None:
                    break
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

        # try parsing with unknowns if no pairwise connections remain (eg. adjectives and UNK_E)
        simple_genlex_fired = False
        if not pairwise_joins:
            # print "\tparsing with simple genlex"  # DEBUG
            for i in range(0, len(partial)):
                if partial[i] is not None:
                    continue
                # proceed only if no meanings for token known (should be relaxed eventually; beam can handle exp)
                if (len(self.lexicon.entries[self.lexicon.surface_forms.index(tree[i][1])]) == 0
                        or tree[i][1] in self.new_adjectives):
                    for d in [-1, 1]:
                        candidate_assignments = self.genlex_for_missing_entry(
                            partial, i, d, tree[i][1], allow_UNK_E=allow_UNK_E)
                        for candidate_assignment, syn_idx in candidate_assignments:
                            possible_joins.append([i, i, candidate_assignment, tree[i], tree[i], syn_idx])
                            simple_genlex_fired = True

        # use generator to do soft alignment and add new lexical entries if no simple unknowns were found
        generator_genlex_succeeded = False
        generator_genlex_entries = []
        if not pairwise_joins and not simple_genlex_fired and generator is not None:
            # print "\tparsing with generator genlex"  # DEBUG
            pt = []
            skip_generator_genlex = False
            for tree_part in tree:
                toks = self.tokens_of_parse_tree(tree_part, allow_None=True, collapse_adj=True, disallow_None_adj=True)
                if type(toks) is bool and not toks:  # found a known adjective assigned None
                    skip_generator_genlex = True
                pt.append(toks)
            if None in pt and not skip_generator_genlex:
                if len(known_tokens) != len(
                        self.flatten([t.split() if type(t) is str else t for t in self.flatten(pt)])):
                    sys.exit("ERROR: extracted bracketing sequence "+str(pt)+" of length "+str(self.flatten(pt)) +
                             " for token sequence "+str(known_tokens)+" of length "+str(len(known_tokens)) +
                             " from trees\n"+self.print_semantic_parse_result(tree))
                target_forms_idx = [partial[i] if partial[i] is not None else False for i in range(0, len(partial))]
                target_forms = [self.lexicon.semantic_forms[target_forms_idx[i]]
                                if type(target_forms_idx[i]) is int else target_forms_idx[i]
                                for i in range(0, len(target_forms_idx))]
                # print "using generator to search for gap-filling entry in "+str(pt)+" with tree " +\
                #     self.print_semantic_parse_result(tree)  # DEBUG
                gen_result = generator.get_token_sequences_for_semantic_form(
                    known_root, target_forms=target_forms, n=sys.maxint, score="both")
                # print "\tgenerated candidate "+str(gen_result)  # DEBUG
                if len(gen_result) > 0:
                    if len(gen_result) != len(pt):
                        sys.exit("ERROR: generated sequence of "+str(len(gen_result))+" partials for " +
                                 "known bracketing sequence "+str(len(pt)))
                    tok_idx = 0
                    for i in range(0, len(pt)):
                        if pt[i] is None:
                            # TODO: draw not from known_tokens[tok_idx] but span of None-assigned tokens here
                            tok_forms = [self.lexicon.semantic_forms[j] for j in
                                         self.lexicon.entries[self.lexicon.surface_forms.index(known_tokens[tok_idx])]]
                            if (gen_result[i] not in tok_forms and  # entry does not already exist
                                    # it is not the case that there is a DESC in the form and the word in question
                                    # already has a lexical entry (ie. can't learn synonyms of DESC that are already
                                    # words in the lexicon)
                                    not (self.lexicon.form_contains_DESC_predicate(gen_result[i]) and
                                         len(self.lexicon.entries[
                                             self.lexicon.surface_forms.index(known_tokens[tok_idx])]) > 0)):
                                new_lex = [known_tokens[tok_idx]+" :- "+self.print_parse(gen_result[i], True, False)]
                                print "new lexicon entry: "+new_lex[0]  # DEBUG
                                self.lexicon.expand_lex_from_strs(new_lex, self.lexicon.surface_forms,
                                                                  self.lexicon.semantic_forms, self.lexicon.entries,
                                                                  self.lexicon.pred_to_surface, False)
                                generator_genlex_entries.append(
                                    [self.lexicon.surface_forms.index(known_tokens[tok_idx]),
                                     self.lexicon.semantic_forms.index(gen_result[i])])
                                print "\texpanded lexicon"  # DEBUG
                                self.lexicon.update_support_structures()
                                print "\tupdated support structures"  # DEBUG
                                generator_genlex_succeeded = True
                            tok_idx += 1
                        elif type(pt[i]) is str:
                            tok_idx += 1
                        else:
                            tok_idx += len(pt[i])

        if generator_genlex_succeeded:  # if we added to lexicon, get those additions
            print "\tre-expanding partials with new genlex addition"  # DEBUG
            expanded_partials = self.expand_partial_parse(p, allow_UNK_E, None)
            print "\tdone; got "+str(len(expanded_partials))+" initial expanded partials"  # DEBUG
            if len(expanded_partials) == 0:
                print "\tremoving added entry from lexicon since it didn't help"  # DEBUG
                for sur_idx, sem_idx in generator_genlex_entries:
                    if sem_idx in self.lexicon.entries[sur_idx]:
                        del self.lexicon.entries[sur_idx][self.lexicon.entries[sur_idx].index(sem_idx)]
                self.lexicon.update_support_structures()
        else:
            expanded_partials = []

        # print "\tperforming joins"  # DEBUG
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
    def genlex_for_missing_entry(self, partial, idx, d, token, allow_UNK_E):

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
            if allow_UNK_E and n_cat[2] == self.lexicon.categories.index('N'):
                # 'N' is a candidate to receive UNK token
                unk = self.create_unk_node()
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
                # print self.ontology.preds  # DEBUG
                # print self.ontology.entries  # DEBUG
                self.ontology.num_args.append(1)
                new_lex = [token + " :- DESC : " + token]
                # print "adding adjective " + str(new_lex)  # DEBUG
                self.lexicon.expand_lex_from_strs(new_lex, self.lexicon.surface_forms,
                                                  self.lexicon.semantic_forms, self.lexicon.entries,
                                                  self.lexicon.pred_to_surface, False)
                self.lexicon.update_support_structures()
                # print "surface forms: "+str(self.lexicon.surface_forms)  # DEBUG
                # print "entries: "+str(self.lexicon.entries)  # DEBUG
                self.new_adjectives.append(token)
            adj_idx = self.lexicon.entries[self.lexicon.surface_forms.index(token)][0]
            candidates.append([self.lexicon.semantic_forms[adj_idx], adj_idx])

        return candidates

    # instantiate a new UNK node and return it
    def create_unk_node(self):
        unk = SemanticNode.SemanticNode(None, None, None, False, idx=self.ontology.preds.index('UNK_E'))
        unk.type = self.ontology.types.index('e')
        unk.set_category(self.lexicon.categories.index('N'))
        unk.set_return_type(self.ontology)
        return unk

    # return DESC : pred raised to N/N : lambda x.(pred(x))
    def perform_adjectival_type_raise(self, A):
        # print "performing adjectival raise with '"+self.print_parse(A,True)+"'" #DEBUG
        child_pred = copy.deepcopy(A)
        child_pred.children = [SemanticNode.SemanticNode(child_pred, self.ontology.types.index('e'),
                                                         self.lexicon.categories.index('N'),
                                                         is_lambda=True, lambda_name=1,
                                                         is_lambda_instantiation=False)]
        child_pred.set_category(self.lexicon.categories.index('N'))  # change from DESC return type
        child_pred.set_return_type(self.ontology)
        raised = SemanticNode.SemanticNode(None, None, None, True, lambda_name=1, is_lambda_instantiation=True)
        raised.type = self.ontology.types.index('e')
        raised.set_category(self.lexicon.categories.index(
            [self.lexicon.categories.index('N'), 1, self.lexicon.categories.index('N')]))
        raised.children = [child_pred]
        child_pred.parent = raised
        raised.set_return_type(self.ontology)
        # print "performed adjectival raise with '"+self.print_parse(A,True)+"' to form '"+self.print_parse(raised,True)+"'" #DEBUG
        if self.safety and not raised.validate_tree_structure():
            sys.exit("ERROR: invalidly linked structure generated by FA: "+
                     self.print_parse(raised, True))
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
            A_B_merged.set_category(A.category)
            innermost_outer_lambda = A_B_merged
            A_child = A.children[0]
            B_child = B.children[0]
            while (innermost_outer_lambda.children != None and innermost_outer_lambda.children[0].is_lambda
                   and innermost_outer_lambda.children[0].is_lambda_instantiation):
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
            innermost_outer_lambda.children[0].children[0].parent = innermost_outer_lambda.children[0]
            innermost_outer_lambda.children[0].children[1].parent = innermost_outer_lambda.children[0]
        else:
            try:
                A.set_return_type(self.ontology)
                B.set_return_type(self.ontology)
            except TypeError:
                raise TypeError("Non-matching child/parent relationship for one of two nodes " +
                                self.print_parse(A, True) + " , " + self.print_parse(B, True))
            if A.return_type != B.return_type:
                sys.exit("performing Merge with '"+self.print_parse(A,True)+"' taking '"+self.print_parse(B,True)+\
                         "' generated mismatched return types"+\
                         self.ontology.compose_str_from_type(A.return_type)+","+\
                         self.ontology.compose_str_from_type(B.return_type))
            input_type = [A.return_type, A.return_type]
            if input_type not in self.ontology.types:
                self.ontology.types.append(input_type)
            full_type = [A.return_type, self.ontology.types.index(input_type)]
            if full_type not in self.ontology.types:
                self.ontology.types.append(full_type)
            A_B_merged = SemanticNode.SemanticNode(None, self.ontology.types.index(full_type),
                                                   A.category, False, idx=and_idx)
            A_B_merged.children = [copy.deepcopy(A), copy.deepcopy(B)]
            A_B_merged.children[0].parent = A_B_merged
            A_B_merged.children[1].parent = A_B_merged

        A_B_merged.set_return_type(self.ontology)
        A_B_merged.commutative_raise_node(self.commutative_idxs, self.ontology)
        # print "performed Merge with '"+self.print_parse(A,True)+"' taking '"+self.print_parse(B,True)+"' to form '"+self.print_parse(A_B_merged,True)+"'" #DEBUG
        if self.safety and not A_B_merged.validate_tree_structure():
            sys.exit("ERROR: invalidly linked structure generated by FA: "+
                     self.print_parse(A_B_merged, True))
        return A_B_merged

    # return true if A,B can be merged
    def can_perform_merge(self, A, B):
        if A is None or B is None: return False
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
    def perform_fa(self, A, B, renumerate=True):
        if self.safety:
            if not A.validate_tree_structure():  # DEBUG
                sys.exit("WARNING: got invalidly linked node '"+self.print_parse(A)+"'")  # DEBUG
            if not B.validate_tree_structure():  # DEBUG
                sys.exit("WARNING: got invalidly linked node '"+self.print_parse(B)+"'")  # DEBUG
        # print "performing FA with '"+self.print_parse(A,True)+"' taking '"+self.print_parse(B,True)+"'"  # DEBUG

        # if A is 'and', apply B to children
        if not A.is_lambda and self.ontology.preds[A.idx] == 'and':
            A_new = copy.deepcopy(A)
            for i in range(0, len(A.children)):
                c = A_new.children[i]
                c_obj = copy.deepcopy(self.lexicon.semantic_forms[c]) if type(c) is int else c
                copy_obj = SemanticNode.SemanticNode(None, None, None, False, 0)
                copy_obj.copy_attributes(c_obj, preserve_parent=True)
                if self.can_perform_type_raising(copy_obj) == "DESC->N/N":
                    copy_obj = self.perform_adjectival_type_raise(c_obj)
                A_new.children[i] = self.perform_fa(copy_obj, B, renumerate=False)
                A_new.children[i].set_return_type(self.ontology)
                A_new.children[i].parent = A_new
            A_new.set_type_from_children_return_types(A_new.children[0].return_type, self.ontology)
            A_new.set_return_type(self.ontology)
            A_new.set_category(A_new.children[0].category)
            A_new.commutative_raise_node(self.commutative_idxs, self.ontology)
            # print "performed FA(1) with '"+self.print_parse(A,True)+"' taking '"+self.print_parse(B,True)+"' to form '"+self.print_parse(A_new,True)+"'" #DEBUG
            if self.safety and not A_new.validate_tree_structure():
                sys.exit("ERROR: invalidly linked structure generated by FA: "+
                         self.print_parse(A_new, True))
            return A_new

        # A is lambda headed and so has a single child which will be the root of the composed tree
        try:
            A_FA_B = copy.deepcopy(A.children[0])
        except TypeError:
            print "A should be lambda but has no children"  # DEBUG
            print "A: "+self.print_parse(A, True)  # DEBUG
            print "B: "+self.print_parse(B, True)  # DEBUG
        A_FA_B = copy.deepcopy(A.children[0])
        A_FA_B.parent = None
        # traverse A_FA_B and replace references to lambda_A with B
        A_FA_B_deepest_lambda = A.lambda_name
        to_traverse = [[A_FA_B, A_FA_B_deepest_lambda]]
        while len(to_traverse) > 0:
            [curr, deepest_lambda] = to_traverse.pop()
            entire_replacement = False
            if curr.is_lambda and curr.is_lambda_instantiation:
                # print "detected deeper lambda "+str(curr.lambda_name)  # DEBUG
                deepest_lambda = curr.lambda_name
            elif curr.is_lambda and not curr.is_lambda_instantiation and curr.lambda_name == A.lambda_name:  # an instance of lambda_A to be replaced by B
                # print "substituting '"+self.print_parse(B,True)+"' for '"+self.print_parse(curr, True)+"' with lambda offset "+str(deepest_lambda) #DEBUG
                if (not B.is_lambda and self.ontology.preds[B.idx] == 'and'
                        and curr.children is not None and B.children is not None):
                    # print "entering B substitution of curr taking curr's args"  # DEBUG
                    # if B is 'and', can preserve it and interleave A's children as arguments
                    raised = False
                    B_new = copy.deepcopy(B)
                    for i in range(0, len(B_new.children)):
                        c = B_new.children[i]
                        c.parent = None
                        c_obj = copy.deepcopy(self.lexicon.semantic_forms[c]) if type(c) is int else c
                        if self.can_perform_type_raising(c_obj) == "DESC->N/N":
                            c_obj = self.perform_adjectival_type_raise(c_obj)
                            raised = True
                        B_new.children[i] = self.perform_fa(c_obj, curr.children[0], renumerate=False)
                        B_new.children[i].increment_lambdas(inc=deepest_lambda)
                        B_new.children[i].parent = curr
                        B_new.children[i].set_return_type(self.ontology)
                    if curr.parent.is_lambda_instantiation:  # eg. 1(2) taking and(pred,pred) -> and(pred(2),pred(2)
                        B_new_arg = B_new.children[0]
                        while self.ontology.preds[B_new_arg.idx] == 'and':
                            # print "B_new_arg: "+self.print_parse(B_new_arg)  # DEBUG
                            B_new_arg = B_new_arg.children[0]
                        # print "setting curr parent lambda name to child of "+self.print_parse(B_new_arg)  # DEBUG
                        curr.parent.lambda_name = B_new_arg.children[0].lambda_name
                    if raised:
                        B_new.set_category(self.lexicon.categories.index(
                            [self.lexicon.categories.index('N'), 1, self.lexicon.categories.index('N')]))
                    curr.copy_attributes(B_new, preserve_parent=True)
                    curr.set_type_from_children_return_types(curr.children[0].return_type, self.ontology)
                    curr.set_return_type(self.ontology)
                    # print "created 'and' consumption result "+self.print_parse(curr, True)  # DEBUG
                elif curr.parent is None:
                    # print "entering None parent for curr"  # DEBUG
                    if curr.children is None:
                        # print "...whole tree is instance" #DEBUG
                        curr.copy_attributes(B)  # instance is whole tree; add nothing more and loop will now exit
                    elif B.children is None:
                        # print "...instance heads tree; preserve children taking B"
                        curr.copy_attributes(B, deepest_lambda, preserve_children=True)
                    else:
                        sys.exit("Error: incompatible parentless, childed node A with childed node B")
                    entire_replacement = True
                    curr.set_category(self.lexicon.categories[A.category][0])  # take on return type of A
                    curr.set_return_type(self.ontology)
                else:
                    # print "entering standard implementation for curr"  # DEBUG
                    for curr_parent_matching_idx in range(0, len(curr.parent.children)):
                        if not curr.parent.children[curr_parent_matching_idx] != curr:  # find matching address
                            break
                    if curr.children is None:
                        # print "...instance of B ("+self.print_parse(B)+") will preserve its children" #DEBUG
                        curr.parent.children[curr_parent_matching_idx].copy_attributes(B, deepest_lambda,
                                                                                       preserve_parent=True)  # lambda instance is a leaf
                        if not curr.parent.children[curr_parent_matching_idx].validate_tree_structure():  # DEBUG
                            sys.exit("ERROR: copy operation produced invalidly linked tree "+
                                     self.print_parse(curr.parent.children[curr_parent_matching_idx], True))  # DEBUG
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
                    curr.parent.children[curr_parent_matching_idx].set_return_type(self.ontology)
            if not entire_replacement and curr.children is not None: to_traverse.extend([[c, deepest_lambda] for c in curr.children])
        if renumerate:
            # print "renumerating "+self.print_parse(A_FA_B)  # DEBUG
            A_FA_B.renumerate_lambdas([])
        try:
            A_FA_B.set_return_type(self.ontology)
        except TypeError as e:
            print e
            sys.exit("ERROR in form '"+self.print_parse(A_FA_B)+"'")
        A_FA_B.set_category(self.lexicon.categories[A.category][0])
        A_FA_B.commutative_raise_node(self.commutative_idxs, self.ontology)
        # print "performed FA(2) with '"+self.print_parse(A,True)+"' taking '"+self.print_parse(B,True)+"' to form '"+self.print_parse(A_FA_B,True)+"'" #DEBUG
        if self.safety and not A_FA_B.validate_tree_structure():
            sys.exit("ERROR: invalidly linked structure generated by FA: " +
                     self.print_parse(A_FA_B, True))
        return A_FA_B

    # return true if A(B) is a valid for functional application
    def can_perform_fa(self, i, j, A, B):
        if A is None or B is None:
            # print "A or B is None"  # DEBUG
            return False
        if A.category is None or type(self.lexicon.categories[A.category]) is not list or (
                            i - j > 0 and self.lexicon.categories[A.category][1] == 1) or (
                            i - j < 0 and self.lexicon.categories[A.category][1] == 0):
            # print "B is left/right when A expects right/left"  # DEBUG
            return False  # B is left/right when A expects right/left
        if not A.is_lambda or not A.is_lambda_instantiation or A.type != B.return_type:
            if A.children is None:  # DEBUG
                sys.exit("ERROR: found lambda with no children: "+str(self.print_parse(A)))
            # print "A is not lambda instantiation or types are mismatched"  # DEBUG
            return False
        if self.lexicon.categories[A.category][2] != B.category:
            # print "B category does not match A expected consumption"  # DEBUG
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
        # print "checking whether '"+self.print_parse(A)+"' lambda "+str(lambda_name) + \
        #       " instances can be replaced by '"+self.print_parse(B)+"' under contexts " + \
        #       str(A_lambda_context)+","+str(B_lambda_context)  # DEBUG
        if A.is_lambda and A.is_lambda_instantiation:
            extended_context = A_lambda_context[:]
            extended_context.append(A.type)
            return self.lambda_value_replacements_valid(A.children[0], lambda_name, extended_context, B, B_lambda_context)
        if A.is_lambda and not A.is_lambda_instantiation and A.lambda_name == lambda_name:  # this is an instance of A's lambda to be replaced by B
            if A.children is not None and B.children is not None:  # the instance takes arguments
                if (len(A_lambda_context) - len(B_lambda_context) == 1 and
                   B.idx == self.ontology.preds.index('and')):
                    return True  # special 'and' consumption rule (A consumes B 'and' and B takes A's arguments)
                if len(A_lambda_context) != len(B_lambda_context):
                    # print "contexts differ in length"  # DEBUG
                    return False  # the lambda contexts differ in length
                matches = [1 if A_lambda_context[i] == B_lambda_context[i] else 0 for i in
                           range(0, len(A_lambda_context))]
                if sum(matches) != len(A_lambda_context):
                    # print "contexts differ in content"  # DEBUG
                    return False  # the lambda contexts differ in content
                return True
            return True  # ie A, B have no children
        if A.children is None:
            return True
        valid_through_children = True
        for c in A.children:
            valid_through_children = self.lambda_value_replacements_valid(c, lambda_name, A_lambda_context, B,
                                                                       B_lambda_context)
            if not valid_through_children:
                break
        return valid_through_children

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
        s = s.replace('?', '')
        s = s.replace("it's", "it is")
        s = s.replace("'s", " 's")
        str_parts = s.split()
        for i in range(0, len(str_parts)):
            if str_parts[i] == "an": str_parts[i] = "a"
        return [p for p in str_parts if len(p) > 0]
