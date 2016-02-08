__author__ = 'jesse'

import random
import math
import operator
import sys
import copy
import ParseNode
import SemanticNode


class Parameters:
    def __init__(self, ont, lex):
        self.ontology = ont
        self.lexicon = lex

        # get initial count data structures
        self._CCG_given_token_counts = self.init_ccg_given_token()
        self._CCG_production_counts = self.init_ccg_production()
        self._lexicon_entry_given_token_counts = self.init_lexicon_entry()

        # calculate probability tables from counts
        # note that this needs to happen every time counts are updated
        self.CCG_given_token = {}
        self.CCG_production = {}
        self.lexicon_entry_given_token = {}
        self.update_probabilities()

    # update the probability tables given counts
    def update_probabilities(self):

        for cat_idx in range(0, len(self.lexicon.categories)):

            # get -log probabilities for CCG_given_token ( P(ccg|surface), P(ccg=c|S)=1 )
            mass = float(sum([self._CCG_given_token_counts[(cat_idx, sf_idx)]
                         for sf_idx in range(0, len(self.lexicon.surface_forms))
                              if (cat_idx, sf_idx) in self._CCG_given_token_counts]))
            for sf_idx in range(0, len(self.lexicon.surface_forms)):
                key = (cat_idx, sf_idx)
                if key in self._CCG_given_token_counts:
                    self.CCG_given_token[key] = -math.log(self._CCG_given_token_counts[key] / mass) \
                        if mass > 0 and self._CCG_given_token_counts[key] > 0 else -sys.maxint

                # get -log probabilities for lexicon_entry_given_token
                # ( P(sem|surface), P(sem=s|S)=1 )
                entry_mass = float(sum([self._lexicon_entry_given_token_counts[(sem_idx, sf_idx)]
                                        for sem_idx in range(0, len(self.lexicon.entries[sf_idx]))
                                        if (sem_idx, sf_idx) in self._lexicon_entry_given_token_counts]))
                for sem_idx in range(0, len(self.lexicon.entries[sf_idx])):
                    key = (sem_idx, sf_idx)
                    if key in self._lexicon_entry_given_token_counts:
                        self.lexicon_entry_given_token[key] = -math.log(self._lexicon_entry_given_token_counts[key]
                                                                        / entry_mass) \
                            if entry_mass > 0 and self._lexicon_entry_given_token_counts[key] > 0 else -sys.maxint

            # get -log probabilities for CCG_production ( P(ccg|(leftcgg, rightccg)), P(ccg=c|LR)=1 )
            mass = float(sum([self._CCG_production_counts[(cat_idx, l_idx, r_idx)]
                              for l_idx in range(0, len(self.lexicon.categories))
                              for r_idx in range(0, len(self.lexicon.categories))
                              if (cat_idx, l_idx, r_idx) in self._CCG_production_counts]))
            for l_idx in range(0, len(self.lexicon.categories)):
                for r_idx in range(0, len(self.lexicon.categories)):
                    key = (cat_idx, l_idx, r_idx)
                    if key in self._CCG_production_counts:
                        self.CCG_production[key] = -math.log(self._CCG_production_counts[key] / mass) \
                            if mass > 0 and self._CCG_production_counts[key] > 0 else -sys.maxint

    # indexed by (categories idx, surface forms idx), value parameter weight
    def init_ccg_given_token(self):
        return {(self.lexicon.semantic_forms[sem_idx].category, sf_idx): 0.0
                for sf_idx in range(0, len(self.lexicon.entries))
                for sem_idx in self.lexicon.entries[sf_idx]}

    # indexed by categories idxs (production, left, right), value parameter weight
    def init_ccg_production(self):
        ccg_production = {}
        for cat_idx in range(0, len(self.lexicon.categories)):

            # add production rules of form Z -> X Y for X(Y)=Z or Y(X)=Z (function application)
            consumables = self.lexicon.find_consumables_for_cat(cat_idx)
            for d, child in consumables:
                if d == 0:  # consumes to the left
                    l = child
                    r = self.lexicon.categories.index([cat_idx, d, child])
                else:  # consumes to the right
                    r = child
                    l = self.lexicon.categories.index([cat_idx, d, child])
                ccg_production[(cat_idx, l, r)] = 0.0

            # add production rules of form X -> X X for X<>X=X (merge)
            ccg_production[(cat_idx, cat_idx, cat_idx)] = 0.0

        return ccg_production

    # indexed by (surface forms idx, semantic forms idx), value parameter weight
    def init_lexicon_entry(self):
        return {(sf_idx, sem_idx): 1.0
                for sf_idx in range(0, len(self.lexicon.entries))
                for sem_idx in self.lexicon.entries[sf_idx]}

    # take in examples t=(x,y,z) for x an expression, y an incorrect semantic form, z a correct semantic form
    def update_learned_parameters(self, t):

        # update counts given new training data
        for x, y, z in t:
            # do perceptron updates of parameters
            parameter_extractors = [count_ccg_surface_form_pairs, count_ccg_productions, count_lexical_entries]
            parameter_structures = [self._CCG_given_token_counts,
                                    self._CCG_production_counts,
                                    self._lexicon_entry_given_token_counts]
            for i in range(0, len(parameter_structures)):
                y_keys = parameter_extractors[i](y)
                z_keys = parameter_extractors[i](z)
                for z_key in z_keys:
                    z_val = z_keys[z_key]
                    y_val = y_keys[z_key] if z_key in y_keys else 0
                    if z_key not in parameter_structures[i]:
                        parameter_structures[i][z_key] = 0
                    parameter_structures[i][z_key] += z_val - y_val

        # update probabilities given new counts
        self.update_probabilities()

        print "CCG_given_token: "+str(self.CCG_given_token)  # DEBUG
        print "CCG_production: "+str(self.CCG_production)  # DEBUG
        print "lexicon_entry: "+str(self.lexicon_entry_given_token)  # DEBUG


# take in ParseNode y to calculate (surface forms idx, semantic forms idx) pairs
def count_lexical_entries(y):
    pairs = {}
    token_assignments = y.get_leaves()
    for ta in token_assignments:
        k = (ta.surface_form, ta.semantic_form)
        if k not in pairs:
            pairs[k] = 0
        pairs[k] += 1
    return pairs


# take in ParseNode y to calculate (CCG, CCG, CCG) productions
def count_ccg_productions(y):
    productions = {}
    to_explore = [y]
    while len(to_explore) > 0:
        n = to_explore.pop()
        if n.children is not None:
            key = (n.node.category, n.children[0].node.category, n.children[1].node.category)
            if key not in productions:
                productions[key] = 0
            productions[key] += 1
            to_explore.extend(n.children)
    return productions


# take in ParseNode y to calculate (CCG, token) pairs
def count_ccg_surface_form_pairs(y):
    pairs = {}
    token_assignments = y.get_leaves()
    for ta in token_assignments:
        k = (ta.node.category, ta.surface_form)
        if k not in pairs:
            pairs[k] = 0
        pairs[k] += 1
    return pairs


class CKYParser:
    def __init__(self, ont, lex):

        # resources given on instantiation
        self.ontology = ont
        self.lexicon = lex

        # model parameter values
        self.theta = Parameters(ont, lex)

        # additional linguistic information
        # TODO: read this from configuration files or have user specify it on instantiation
        self.commutative_idxs = [self.ontology.preds.index('and'), self.ontology.preds.index('or')]
        self.max_multiword_expression = 3

        # behavioral parameters
        self.safety = True  # set to False once confident about node combination functions' correctness

        # cache
        self.cached_combinations = {}  # indexed by left, then right node, value at result

    # print a SemanticNode as a string using the known ontology
    def print_parse(self, p, show_category=False, show_non_lambda_types=False):
        if p is None:
            return "NONE"
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

    # read in data set of form utterance\nCCG : semantic_form\n\n...
    def read_in_paired_utterance_semantics(self, fname, allow_expanding_ont=False):
        d = []
        f = open(fname, 'r')
        f_lines = f.readlines()
        f.close()
        i = 0
        while i < len(f_lines):
            if len(f_lines[i].strip()) == 0:
                i += 1
                continue
            input_str = f_lines[i].strip()
            ccg_str, form_str = f_lines[i+1].strip().split(" : ")
            ccg = self.lexicon.read_category_from_str(ccg_str)
            form = self.lexicon.read_semantic_form_from_str(form_str, None, None, [],
                                                            allow_expanding_ont=allow_expanding_ont)
            form.category = ccg
            d.append([input_str, form])
            i += 3
        return d

    # take in a data set D=(x,y) for x expressions and y correct semantic form and update CKYParser parameters
    def train_learner_on_semantic_forms(self, d, epochs=10):
        for e in range(0, epochs):
            print "epoch " + str(e)  # DEBUG
            t, failures = self.get_training_pairs(d)
            if len(t) == 0:
                print "training converged at epoch " + str(e)
                if failures == 0:
                    return True
                else:
                    return False
            random.shuffle(t)
            self.theta.update_learned_parameters(t)
        return False

    # take in data set d=(x,y) for x strings and y correct semantic forms and calculate training pairs
    def get_training_pairs(self, d):
        t = []
        num_trainable = 0
        num_matches = 0
        num_fails = 0
        for [x, y] in d:
            correct_parse = None
            parse_generator = self.most_likely_cky_parse(x, known_root=y)
            chosen_parse = next(parse_generator)
            current_parse = chosen_parse
            match = False
            first = True
            if chosen_parse is None:
                print "WARNING: could not find valid parse for '" + x + "' during training"
                num_fails += 1
                continue
            while correct_parse is None and current_parse is not None:
                if y.equal_allowing_commutativity(
                        current_parse.node, self.commutative_idxs, ontology=self.ontology):
                    correct_parse = current_parse
                    if first:
                        match = True
                        num_matches += 1
                    else:
                        num_trainable += 1
                    break
                first = False
                current_parse = next(parse_generator)
            if correct_parse is None:
                print "WARNING: could not find correct parse for input sequence "+str(x)
                num_fails += 1
                continue
            if not match:
                print "training example generated:"  # DEBUG
                print "x: "+str(x)  # DEBUG
                print "chosen_parse: "+self.print_parse(chosen_parse.node, show_category=True)  # DEBUG
                print "correct_parse: "+self.print_parse(correct_parse.node, show_category=True)  # DEBUG
                print "y: "+self.print_parse(y, show_category=True)  # DEBUG
                t.append([x, chosen_parse, correct_parse])
        print "matched "+str(num_matches)+"/"+str(len(d))  # DEBUG
        print "trained "+str(num_trainable)+"/"+str(len(d))  # DEBUG
        print "failed "+str(num_fails)+"/"+str(len(d))  # DEBUG
        return t, num_fails

    # yields the next most likely CKY parse of tokens tks
    # if the root of the tree is known (during supervised training, for example),
    # providing it as an argument to this method allows top-down generation
    # to find new lexical entries for surface forms not yet recognized
    def most_likely_cky_parse(self, s, known_root=None):

        tk_seqs = self.tokenize(s)
        # print "tk_seqs: "+str(tk_seqs)  # DEBUG

        ccg_parse_tree_generator = self.most_likely_ccg_parse_tree(tk_seqs)
        # get next most likely CCG parse tree out of CKY algorithm
        ccg_tree, tree_score, tks = next(ccg_parse_tree_generator)
        # ccg_tree indexed by spans (i, j) valued at [CCG category, left span, right span]
        while ccg_tree is not None:

            # print "ccg tree: "  # DEBUG
            # for span in ccg_tree:  # DEBUG
            #     print str(span) + ": [" + self.lexicon.compose_str_from_category(ccg_tree[span][0]) + \
            #         "," + str(ccg_tree[span][1]) + "," + str(ccg_tree[span][2]) + "]"  # DEBUG

            # if we know the root, we can dismiss ccg trees with the wrong root syntax without further computation
            if known_root is None or ccg_tree[(0, len(tks))][0] == known_root.category:

                semantic_assignment_generator = self.most_likely_semantic_leaves(tks, ccg_tree)
                # get next most likely assignment of semantics to given CCG categories
                parse_leaves, leaves_score = next(semantic_assignment_generator)
                while parse_leaves is not None:

                    parse_tree_generator = self.most_likely_tree_generator(parse_leaves,
                                                                           ccg_tree, sem_root=known_root)
                    parse_tree = next(parse_tree_generator)
                    while parse_tree is not None:
                        yield parse_tree

                    parse_leaves, leaves_score = next(semantic_assignment_generator)

            ccg_tree, tree_score, tks = next(ccg_parse_tree_generator)

        # out of parse trees to try
        yield None

    # yields the next most likely assignment of semantic values to ccg nodes
    # returns None if leaf assignments cannot propagate to root
    # returns None if no assignment to missing leaf entries will allow propagation to root
    def most_likely_tree_generator(self, parse_leaves, ccg_tree, sem_root=None):

        num_tks = len(parse_leaves)
        parse_leaves, parse_leaves_keys = self.form_root_from_leaves(parse_leaves, ccg_tree)
        # print "parse_leaves: "+str(parse_leaves)  # DEBUG

        # yield parse and total score (leaves+syntax) if structure matches
        if len(parse_leaves) == 1:
            yield parse_leaves[0]  # the ParseNode root of the finished parse tree

        # if structure does not match and there are None nodes, perform top-down generation from
        # supervised root (if available) to fill in unknown semantic gaps given CKY structure of
        # the ccg_tree
        parse_leaves_nodes = [pl.node for pl in parse_leaves]
        if sem_root is not None and None in parse_leaves_nodes:

            top_down_chart = {}
            root_key = (0, num_tks)
            top_down_chart[root_key] = ParseNode.ParseNode(None, sem_root)

            topdown_tree_generator = self.get_most_likely_tree_from_root(top_down_chart[root_key],
                                                                         root_key,
                                                                         ccg_tree,
                                                                         parse_leaves_keys)
            topdown_root, topdown_score = next(topdown_tree_generator)
            while topdown_root is not None:

                new_lex_entries = []  # values (surface form str, sem node)
                topdown_leaves = topdown_root.get_leaves()
                if len(topdown_leaves) == len(parse_leaves):  # possible match was found in reverse parsing
                    match = True
                    candidate_parse_leaves = parse_leaves[:]
                    for idx in range(0, len(candidate_parse_leaves)):
                        if candidate_parse_leaves[idx].node is None:
                            new_lex_entries.append([candidate_parse_leaves[idx].surface_form, topdown_leaves[idx].node])
                            candidate_parse_leaves[idx] = topdown_leaves[idx]
                            continue
                        if candidate_parse_leaves[idx].node.category != topdown_leaves[idx].node.category:
                            match = False
                            break
                        if not candidate_parse_leaves[idx].node.equal_allowing_commutativity(topdown_leaves[idx].node,
                                                                                             commutative_idxs=
                                                                                             self.commutative_idxs):
                            match = False
                            break
                    if match:
                        print "new_lex_entries: "  # DEBUG
                        for nle in new_lex_entries:
                            print nle[0]+" :- "+self.print_parse(nle[1], show_category=True)  # DEBUG
                        # TODO: add these to lexicon and print warning of some kind, then update parameter maps
                        candidate_parse_leaves = self.form_root_from_leaves(candidate_parse_leaves, ccg_tree)

                        if len(candidate_parse_leaves) == 1:
                            yield candidate_parse_leaves[0]  # the ParseNode root of the finished parse tree

                topdown_root, topdown_score = next(topdown_tree_generator)

        yield None

    # given parse leaves, form as close to root as possible
    def form_root_from_leaves(self, parse_leaves, ccg_tree):

        # try to build parse tree from lexical assignments guided by CKY structure
        spans = [(i, i+1) for i in range(0, len(parse_leaves))]
        found_combination = True
        while len(parse_leaves) > 1 and found_combination:

            # print "parse leaves: "  # DEBUG
            # print parse_leaves  # DEBUG
            # for i in range(0, len(parse_leaves)):
            #     print str(i)+": " + self.print_parse(parse_leaves[i].node, show_category=True) \
            #         if parse_leaves[i] is not None else str(None)  # DEBUG

            found_combination = False
            for i in range(0, len(parse_leaves)-1):
                root_span = (spans[i][0], spans[i+1][1])
                if root_span in ccg_tree:  # these two leaves must combine given tree

                    # print "investigating combination at "+str(root_span)  # DEBUG
                    if parse_leaves[i].node is None or parse_leaves[i+1].node is None:
                        continue

                    l = parse_leaves[i].node
                    r = parse_leaves[i+1].node
                    if (l in self.cached_combinations and
                       r in self.cached_combinations[l]):
                        root = self.cached_combinations[l][r]
                    else:
                        root = None
                        if self.can_perform_fa(i, i+1, l, r):
                            root = self.perform_fa(l, r)
                        elif self.can_perform_fa(i+1, i, r, l):
                            root = self.perform_fa(r, l)
                        elif self.can_perform_merge(l, r):
                            root = self.perform_merge(l, r)
                        elif self.can_perform_merge(r, l):
                            root = self.perform_merge(r, l)
                        if root is not None:
                            if i not in self.cached_combinations:
                                self.cached_combinations[i] = {}
                            self.cached_combinations[i][i+1] = root

                    if root is not None:
                        joined = ParseNode.ParseNode(None, root,
                                                     children=[parse_leaves[i], parse_leaves[i+1]])
                        parse_leaves[i] = joined
                        del parse_leaves[i+1]
                        spans[i] = (spans[i][0], spans[i+1][1])
                        del spans[i+1]
                        found_combination = True  # start iteration over since we modified list
                        # print "found combination at "+str(root_span)  # DEBUG
                        break
        return parse_leaves, spans

    # greedily yields the next most likely tree generated from given parse root
    # subject to the constraints of the ccg_tree and stopping upon reaching all known_leaf_keys
    def get_most_likely_tree_from_root(self, parse_root, root_key, ccg_tree, known_leaf_keys):

        # greedily take children with best score until we get a category match
        children_generator = self.get_most_likely_children_from_root(parse_root.node)
        children, children_score = next(children_generator)
        while children is not None:
            # when we find children with syntax matching ccg tree, save and see whether either should expand
            if children[0].category == ccg_tree[ccg_tree[root_key][1]][0] and \
               children[1].category == ccg_tree[ccg_tree[root_key][2]][0]:
                parse_root.children = []
                for c in range(0, 2):  # save children
                    parse_root.children.append(ParseNode.ParseNode(parse_root, children[c]))
                if ccg_tree[root_key][1] in known_leaf_keys and ccg_tree[root_key][2] in known_leaf_keys:
                    yield parse_root, children_score
                for c in range(0, 2):  # expand children
                    subtree_generators = [self.get_most_likely_tree_from_root(parse_root.children[c],
                                                                              ccg_tree[root_key][1+c],
                                                                              ccg_tree,
                                                                              known_leaf_keys)
                                          for c in range(0, 2)]
                    if ccg_tree[root_key][1+c] not in known_leaf_keys:
                        for child1, score1 in subtree_generators[c]:
                            if child1 is None:  # could not continue down tree
                                break
                            parse_root.children[c] = child1
                            if ccg_tree[root_key][1+((c+1) % 2)] not in known_leaf_keys:
                                for child2, score2 in subtree_generators[(c+1) % 2]:
                                    if child2 is None:  # could not continue down tree
                                        break
                                    parse_root.children[(c+1) % 2] = child2
                                    yield parse_root, children_score+score1+score2
                            else:
                                yield parse_root, children_score+score1

            children, children_score = next(children_generator)

        yield None, -sys.maxint

    # yields next most likely pair of children from a given semantic root using production rule parameter scores
    def get_most_likely_children_from_root(self, n):

        candidate_pairs = self.perform_reverse_fa(n)
        if self.can_perform_split(n):
            candidate_pairs.extend(self.perform_split(n))
        match_scores = {}  # indexed by candidate_pair idx, value score
        for prod in self.theta.CCG_production:
            if prod[0] == n.category:
                for pair_idx in range(0, len(candidate_pairs)):
                    pair = candidate_pairs[pair_idx]
                    if ((pair[1] == 1 and prod[1] == pair[0].category and prod[2] == pair[2].category)
                       or (pair[1] == 0 and prod[1] == pair[2].category and prod[2] == pair[0].category)):
                        match_scores[pair_idx] = self.theta.CCG_production[prod]
        for pair_idx, score in sorted(match_scores.items(), key=operator.itemgetter(1), reverse=True):
            children = [candidate_pairs[pair_idx][0], candidate_pairs[pair_idx][2]] \
                if candidate_pairs[pair_idx][1] == 1 else \
                [candidate_pairs[pair_idx][2], candidate_pairs[pair_idx][0]]
            yield children, score
        yield None, -sys.maxint

    # yields next most likely assignment of semantic values to ccg tree leaves
    def most_likely_semantic_leaves(self, tks, ccg_tree):

        # get syntax categories for tree leaves
        leaf_categories = [ccg_tree[(idx, idx+1)][0] for idx in range(0, len(tks))]

        # get possible semantic forms for each syntax/surface combination represented by leaf_categories and tks
        semantic_candidates = []
        for idx in range(0, len(tks)):
            if tks[idx] in self.lexicon.surface_forms:
                semantic_candidates.append([sem_idx
                                            for sem_idx
                                            in self.lexicon.entries[self.lexicon.surface_forms.index(tks[idx])]
                                            if self.lexicon.semantic_forms[sem_idx].category == leaf_categories[idx]])
            else:  # unknown surface form
                semantic_candidates.append([])

        # print "semantic_candidates: "+str(semantic_candidates)  # DEBUG

        # score all possible assignments of semantics to given surface forms and syntax categories
        scores = {}  # indexed by tuple of semantic form idxs
        i = 0
        finished = False
        while not finished:
            finished = True
            curr = i
            assignments = []
            score = 0
            for idx in range(0, len(tks)):
                if len(semantic_candidates[idx]) == 0:
                    assignments.append(None)  # no known semantic assignment
                else:
                    assignment_idx = curr % len(semantic_candidates[idx])
                    assignments.append(assignment_idx)
                    key = (self.lexicon.surface_forms.index(tks[idx]), semantic_candidates[idx][assignment_idx])
                    score += self.theta.lexicon_entry_given_token[key] \
                        if key in self.theta.lexicon_entry_given_token else -sys.maxint
                    if assignment_idx < len(semantic_candidates[idx])-1:
                        finished = False
                    curr /= len(semantic_candidates[idx])
            i += 1
            scores[tuple(assignments)] = score

        # yield the assignment tuples in order by score as lists of ParseNodes
        for assignment, score in sorted(scores.items(), key=operator.itemgetter(1), reverse=True):
            nodes = [ParseNode.ParseNode(None,
                                         copy.deepcopy(self.lexicon.semantic_forms[
                                                       semantic_candidates[idx][assignment[idx]]])
                                         if assignment[idx] is not None else None,
                                         surface_form=self.lexicon.surface_forms.index(tks[idx])
                                         if assignment[idx] is not None else tks[idx],
                                         semantic_form=semantic_candidates[idx][assignment[idx]]
                                         if assignment[idx] is not None else None)
                     for idx in range(0, len(tks))]
            yield nodes, score

        yield None, -sys.maxint

    # yields the next most likely ccg parse tree given a set of possible token sequences
    def most_likely_ccg_parse_tree(self, tk_seqs):

        ccg_parse_tree_generators = [self.most_likely_ccg_parse_tree_given_tokens(tks) for tks in tk_seqs]
        best_per_seq = [next(ccg_parse_tree_generators[idx]) for idx in range(0, len(tk_seqs))]

        best_idx = 0
        while len(best_per_seq) > 0:
            for idx in range(0, len(tk_seqs)):
                if best_per_seq[idx][1] > best_per_seq[best_idx][1]:
                    best_idx = idx
            yield best_per_seq[best_idx][0], best_per_seq[best_idx][1], tk_seqs[best_idx]
            candidate, score = next(ccg_parse_tree_generators[best_idx])
            if candidate is not None:
                best_per_seq[best_idx] = [candidate, score]
            else:
                del best_per_seq[best_idx]
                del ccg_parse_tree_generators[best_idx]
                del tk_seqs[best_idx]
        yield None, -sys.maxint, None

    # yields the next most likely ccg parse tree given a set of tokens
    def most_likely_ccg_parse_tree_given_tokens(self, tks):

        # print tks  # DEBUG

        # indexed by position in span (i, j) in parse tree
        # value is list of tuples [CCG category, [left key, left index], [right key, right index], score]
        chart = {}

        # values are positions in span (i, j) in parse tree
        # value is present if this span is missing from the lexicon and thus, in the chart, eligible to
        # receive arbitrary matching syntax categories given neighbors during CKY
        missing = []

        # assign leaf values
        for idx in range(0, len(tks)):
            pos = (idx, idx+1)
            chart[pos] = []
            if tks[idx] in self.lexicon.surface_forms:
                sf_idx = self.lexicon.surface_forms.index(tks[idx])
                cats = [self.lexicon.semantic_forms[sem_idx].category for sem_idx in self.lexicon.entries[sf_idx]]
                for cat in cats:
                    score = self.theta.CCG_given_token[(cat, sf_idx)] \
                        if (cat, sf_idx) in self.theta.CCG_given_token else 0
                    chart[pos].append([cat, None, None, score])
            else:
                missing.append(pos)

        # print "leaf chart: "+str(chart)  # DEBUG
        # print "leaf missing: "+str(missing)  # DEBUG
        # _ = raw_input()  # DEBUG

        # populate chart
        for width in range(2, len(tks)+1):
            # print "width: "+str(width)  # DEBUG
            for start in range(0, len(tks)-width+1):
                end = start + width
                key = (start, end)
                if key not in chart:
                    chart[key] = []
                # print "key: "+str(key)  # DEBUG
                for mid in range(start+1, end):
                    l_key = (start, mid)
                    r_key = (mid, end)
                    left = chart[l_key] if l_key in chart else []
                    right = chart[r_key] if r_key in chart else []
                    # print "l_key: "+str(l_key)  # DEBUG
                    # print "r_key: "+str(r_key)  # DEBUG
                    for l_idx in range(0, len(left)):
                        l = left[l_idx]
                        # print "l: "+str(l)+", cat="+self.lexicon.compose_str_from_category(l[0])  # DEBUG
                        for r_idx in range(0, len(right)):
                            r = right[r_idx]
                            # print "r: "+str(r)+", cat="+self.lexicon.compose_str_from_category(r[0])  # DEBUG
                            for prod in self.theta.CCG_production:
                                # print "prod: "+str([self.lexicon.compose_str_from_category(c) for c in prod])  # DEBUG
                                if prod[1] == l[0] and prod[2] == r[0]:
                                    new_score = self.theta.CCG_production[prod] + l[3] + r[3]
                                    chart[key].append([prod[0], [l_key, l_idx], [r_key, r_idx], new_score])
                                    # print "new chart entry "+str(key)+" : "+str(chart[key][-1])  # DEBUG
                    lr_keys = [l_key, r_key]
                    lr_dirs = [left, right]
                    lr_idxs = [1, 2]
                    for branch_idx in range(0, 2):
                        m_idx = branch_idx
                        p_idx = 0 if branch_idx == 1 else 1
                        if lr_keys[m_idx] in missing and lr_keys[p_idx] not in missing:
                            # print "searching for matches for missing "+str(lr_keys[m_idx]) + \
                            #       " against present "+str(lr_keys[p_idx])  # DEBUG
                            # investigate syntax for leaf m that can consume or be consumed by p
                            # because of rewriting the CCG rules in CNF form, if a production rule is present
                            # it means that a consumation or merge can happen and we can be agnostic to how
                            for idx in range(0, len(lr_dirs[p_idx])):
                                p = lr_dirs[p_idx][idx]
                                for prod in self.theta.CCG_production:
                                    # leaf m can be be combine with p
                                    if prod[lr_idxs[p_idx]] == p[0]:
                                        m = [prod[lr_idxs[m_idx]], None, None, 0.0]  # new, even probability entries
                                        chart[lr_keys[m_idx]].append(m)
                                        new_score = self.theta.CCG_production[prod] + p[3]
                                        new_entry = [prod[0], None, None, new_score]
                                        new_entry[lr_idxs[m_idx]] = [lr_keys[m_idx], len(chart[lr_keys[m_idx]])-1]
                                        new_entry[lr_idxs[p_idx]] = [lr_keys[p_idx], idx]
                                        chart[key].append(new_entry)
                                        # print "new chart entry "+str(key)+" : "+str(chart[key][-1])  # DEBUG

                    # print "chart: "+str(chart)  # DEBUG
                    # _ = raw_input()  # DEBUG

        # print "finished chart: "+str(chart)  # DEBUG
        # _ = raw_input()

        # return most likely trees in order at root
        key = (0, len(tks))
        while len(chart[key]) > 0:

            # pop best root
            best_idx = 0
            best = chart[key][0]
            for i in range(1, len(chart[key])):
                if chart[key][i][3] > best[3]:
                    best = chart[key][i][:]
                    best_idx = i

            # build and return tree from this root
            tree = {}  # indexed as spans (i, j) like chart, valued at [prod, left span, right span, score]
            to_add = [[key, best]]
            while len(to_add) > 0:
                new_key, to_expand = to_add.pop()
                tree_add = [to_expand[0]]
                for i in range(1, 3):
                    if to_expand[i] is not None:
                        tree_add.append(to_expand[i][0])
                        to_add.append([to_expand[i][0], chart[to_expand[i][0]][to_expand[i][1]]])
                    else:
                        tree_add.append(None)
                tree[new_key] = tree_add

            # yield the current tree and remove the previous root from the structure
            yield tree, chart[key][best_idx][3]  # return tree and score
            del chart[key][best_idx]

        # no parses left
        yield None, -sys.maxint

    # return A<>B; A and B must have matching lambda headers and syntactic categories to be AND merged
    def perform_merge(self, a, b):
        # print "performing Merge with '"+self.print_parse(a, True)+"' taking '"+self.print_parse(b, True)+"'"  # DEBUG

        and_idx = self.ontology.preds.index('and')

        if a.is_lambda_instantiation:
            ab = copy.deepcopy(a)
            ab.set_category(a.category)
            innermost_outer_lambda = ab
            a_child = a.children[0]
            b_child = b.children[0]
            while (innermost_outer_lambda.children is not None and innermost_outer_lambda.children[0].is_lambda
                   and innermost_outer_lambda.children[0].is_lambda_instantiation):
                innermost_outer_lambda = innermost_outer_lambda.children[0]
                a_child = a.children[0]
                b_child = b.children[0]
            innermost_outer_lambda.children = [
                SemanticNode.SemanticNode(innermost_outer_lambda, self.ontology.entries[and_idx],
                                          innermost_outer_lambda.children[0].category, False, idx=and_idx)]
            innermost_outer_lambda.children[0].children = [copy.deepcopy(a_child), copy.deepcopy(b_child)]

            # 'and' adopts type taking each child's and returning the same
            a_child.set_return_type(self.ontology)
            a_child.set_return_type(self.ontology)
            input_type = [a_child.return_type, a_child.return_type]
            if input_type not in self.ontology.types:
                self.ontology.types.append(input_type)
            full_type = [a_child.return_type, self.ontology.types.index(input_type)]
            if full_type not in self.ontology.types:
                self.ontology.types.append(full_type)
            innermost_outer_lambda.children[0].type = self.ontology.types.index(full_type)
            innermost_outer_lambda.children[0].set_return_type(self.ontology)
            innermost_outer_lambda.children[0].children[0].parent = innermost_outer_lambda.children[0]
            innermost_outer_lambda.children[0].children[1].parent = innermost_outer_lambda.children[0]
        else:
            try:
                a.set_return_type(self.ontology)
                b.set_return_type(self.ontology)
            except TypeError:
                raise TypeError("Non-matching child/parent relationship for one of two nodes " +
                                self.print_parse(a, True) + " , " + self.print_parse(b, True))
            if a.return_type != b.return_type:
                sys.exit("performing Merge with '"+self.print_parse(a, True)+"' taking '"+self.print_parse(a, True) +
                         "' generated mismatched return types" +
                         self.ontology.compose_str_from_type(a.return_type)+"," +
                         self.ontology.compose_str_from_type(b.return_type))
            input_type = [a.return_type, a.return_type]
            if input_type not in self.ontology.types:
                self.ontology.types.append(input_type)
            full_type = [a.return_type, self.ontology.types.index(input_type)]
            if full_type not in self.ontology.types:
                self.ontology.types.append(full_type)
            ab = SemanticNode.SemanticNode(None, self.ontology.types.index(full_type),
                                           a.category, False, idx=and_idx)
            ab.children = [copy.deepcopy(a), copy.deepcopy(b)]
            ab.children[0].parent = ab
            ab.children[1].parent = ab

        ab.set_return_type(self.ontology)
        ab.commutative_raise_node(self.commutative_idxs, self.ontology)
        # print "performed Merge with '"+self.print_parse(a, True)+"' taking '"+self.print_parse(b, True) + \
        #     "' to form '"+self.print_parse(ab, True)+"'"  # DEBUG
        if self.safety and not ab.validate_tree_structure():
            sys.exit("ERROR: invalidly linked structure generated by FA: " +
                     self.print_parse(ab, True))
        return ab

    # return true if A,B can be merged
    def can_perform_merge(self, a, b):
        if a is None or b is None:
            return False
        if self.lexicon.categories[a.category] != self.lexicon.categories[b.category]:
            return False
        if a.return_type is None:
            a.set_return_type(self.ontology)
        if b.return_type is None:
            b.set_return_type(self.ontology)
        if a.return_type != b.return_type:
            return False
        curr_a = a
        curr_b = b
        while curr_a.is_lambda and curr_a.is_lambda_instantiation:
            if curr_a.return_type is None:
                curr_a.set_return_type(self.ontology)
            if curr_b.return_type is None:
                curr_b.set_return_type(self.ontology)
            if (not curr_b.is_lambda or not curr_b.is_lambda_instantiation or curr_b.type != curr_a.type
                    or curr_a.return_type != curr_b.return_type):
                return False
            curr_a = curr_a.children[0]
            curr_b = curr_b.children[0]
        return True

    # return A(B); A must be lambda headed with type equal to B's root type
    def perform_fa(self, a, b, renumerate=True):
        if self.safety:
            if not a.validate_tree_structure():  # DEBUG
                sys.exit("WARNING: got invalidly linked node '"+self.print_parse(a)+"'")  # DEBUG
            if not b.validate_tree_structure():  # DEBUG
                sys.exit("WARNING: got invalidly linked node '"+self.print_parse(b)+"'")  # DEBUG
        # print "performing FA with '"+self.print_parse(a, True)+"' taking '"+self.print_parse(b, True)+"'"  # DEBUG

        # if A is 'and', apply B to children
        if not a.is_lambda and self.ontology.preds[a.idx] == 'and':
            a_new = copy.deepcopy(a)
            for i in range(0, len(a.children)):
                c = a_new.children[i]
                c_obj = copy.deepcopy(self.lexicon.semantic_forms[c]) if type(c) is int else c
                copy_obj = SemanticNode.SemanticNode(None, None, None, False, 0)
                copy_obj.copy_attributes(c_obj, preserve_parent=True)
                # TODO: figure out how to re-integrate once unary rules are re-integrated
                # if self.can_perform_type_raising(copy_obj) == "DESC->N/N":
                #     copy_obj = self.perform_adjectival_type_raise(c_obj)
                a_new.children[i] = self.perform_fa(copy_obj, b, renumerate=False)
                a_new.children[i].set_return_type(self.ontology)
                a_new.children[i].parent = a_new
            a_new.set_type_from_children_return_types(a_new.children[0].return_type, self.ontology)
            a_new.set_return_type(self.ontology)
            a_new.set_category(a_new.children[0].category)
            a_new.commutative_raise_node(self.commutative_idxs, self.ontology)
            # print "performed FA(1) with '"+self.print_parse(a, True)+"' taking '"+self.print_parse(b, True) + \
            #     "' to form '"+self.print_parse(a_new, True)+"'"  # DEBUG
            if self.safety and not a_new.validate_tree_structure():
                sys.exit("ERROR: invalidly linked structure generated by FA: " +
                         self.print_parse(a_new, True))
            return a_new

        # A is lambda headed and so has a single child which will be the root of the composed tree
        ab = copy.deepcopy(a.children[0])
        ab.parent = None
        # traverse A_FA_B and replace references to lambda_A with B
        ab_deepest_lambda = a.lambda_name
        to_traverse = [[ab, ab_deepest_lambda]]
        while len(to_traverse) > 0:
            [curr, deepest_lambda] = to_traverse.pop()
            entire_replacement = False
            if curr.is_lambda and curr.is_lambda_instantiation:
                # print "detected deeper lambda "+str(curr.lambda_name)  # DEBUG
                deepest_lambda = curr.lambda_name
            # an instance of lambda_A to be replaced by B
            elif curr.is_lambda and not curr.is_lambda_instantiation and curr.lambda_name == a.lambda_name:
                # print "substituting '"+self.print_parse(B,True)+"' for '"+self.print_parse(curr, True) + \
                # "' with lambda offset "+str(deepest_lambda) #DEBUG
                if (not b.is_lambda and self.ontology.preds[b.idx] == 'and'
                        and curr.children is not None and b.children is not None):
                    # print "entering B substitution of curr taking curr's args"  # DEBUG
                    # if B is 'and', can preserve it and interleave A's children as arguments
                    raised = False
                    b_new = copy.deepcopy(b)
                    for i in range(0, len(b_new.children)):
                        c = b_new.children[i]
                        c.parent = None
                        c_obj = copy.deepcopy(self.lexicon.semantic_forms[c]) if type(c) is int else c
                        # TODO: figure out how to re-integrate once unary rules are re-integrated
                        # if self.can_perform_type_raising(c_obj) == "DESC->N/N":
                        #     c_obj = self.perform_adjectival_type_raise(c_obj)
                        #     raised = True
                        b_new.children[i] = self.perform_fa(c_obj, curr.children[0], renumerate=False)
                        b_new.children[i].increment_lambdas(inc=deepest_lambda)
                        b_new.children[i].parent = curr
                        b_new.children[i].set_return_type(self.ontology)
                    if curr.parent.is_lambda_instantiation:  # eg. 1(2) taking and(pred,pred) -> and(pred(2),pred(2)
                        b_new_arg = b_new.children[0]
                        while self.ontology.preds[b_new_arg.idx] == 'and':
                            # print "B_new_arg: "+self.print_parse(B_new_arg)  # DEBUG
                            b_new_arg = b_new_arg.children[0]
                        # print "setting curr parent lambda name to child of "+self.print_parse(B_new_arg)  # DEBUG
                        curr.parent.lambda_name = b_new_arg.children[0].lambda_name
                    if raised:
                        b_new.set_category(self.lexicon.categories.index(
                            [self.lexicon.categories.index('N'), 1, self.lexicon.categories.index('N')]))
                    curr.copy_attributes(b_new, preserve_parent=True)
                    curr.set_type_from_children_return_types(curr.children[0].return_type, self.ontology)
                    curr.set_return_type(self.ontology)
                    # print "created 'and' consumption result "+self.print_parse(curr, True)  # DEBUG
                elif curr.parent is None:
                    # print "entering None parent for curr"  # DEBUG
                    if curr.children is None:
                        # print "...whole tree is instance" #DEBUG
                        curr.copy_attributes(b)  # instance is whole tree; add nothing more and loop will now exit
                    elif b.children is None:
                        # print "...instance heads tree; preserve children taking B"
                        curr.copy_attributes(b, deepest_lambda, preserve_children=True)
                    else:
                        sys.exit("Error: incompatible parentless, childed node A with childed node B")
                    entire_replacement = True
                    curr.set_category(self.lexicon.categories[a.category][0])  # take on return type of A
                    curr.set_return_type(self.ontology)
                else:
                    # print "entering standard implementation for curr"  # DEBUG
                    for curr_parent_matching_idx in range(0, len(curr.parent.children)):
                        if not curr.parent.children[curr_parent_matching_idx] != curr:  # find matching address
                            break
                    if curr.children is None:
                        # print "...instance of B ("+self.print_parse(B)+") will preserve its children" #DEBUG
                        # lambda instance is a leaf
                        curr.parent.children[curr_parent_matching_idx].copy_attributes(b, deepest_lambda,
                                                                                       preserve_parent=True)
                        if not curr.parent.children[curr_parent_matching_idx].validate_tree_structure():  # DEBUG
                            sys.exit("ERROR: copy operation produced invalidly linked tree " +
                                     self.print_parse(curr.parent.children[curr_parent_matching_idx], True))  # DEBUG
                    else:
                        if b.children is None:
                            # print "...instance of B will keep children from A" #DEBUG
                            curr.parent.children[curr_parent_matching_idx].copy_attributes(b, deepest_lambda,
                                                                                           preserve_parent=True,
                                                                                           preserve_children=True)
                        else:
                            # print "...instance of A and B have matching lambda headers to be merged" #DEBUG
                            b_without_lambda_headers = b
                            num_leading_lambdas = 0
                            while (b_without_lambda_headers.is_lambda and
                                    b_without_lambda_headers.is_lambda_instantiation):
                                b_without_lambda_headers = b_without_lambda_headers.children[0]
                                num_leading_lambdas += 1
                            curr.parent.children[curr_parent_matching_idx].copy_attributes(b_without_lambda_headers,
                                                                                           num_leading_lambdas,
                                                                                           preserve_parent=True,
                                                                                           preserve_children=False)
                    curr.parent.children[curr_parent_matching_idx].set_return_type(self.ontology)
            if not entire_replacement and curr.children is not None:
                to_traverse.extend([[c, deepest_lambda] for c in curr.children])
        if renumerate:
            # print "renumerating "+self.print_parse(A_FA_B)  # DEBUG
            ab.renumerate_lambdas([])
        try:
            ab.set_return_type(self.ontology)
        except TypeError as e:
            print e
            sys.exit("ERROR in form '"+self.print_parse(ab)+"'")
        ab.set_category(self.lexicon.categories[a.category][0])
        ab.commutative_raise_node(self.commutative_idxs, self.ontology)
        # print "performed FA(2) with '"+self.print_parse(a, True)+"' taking '"+self.print_parse(b, True) + \
        #       "' to form '"+self.print_parse(ab, True)+"'"  # DEBUG
        if self.safety and not ab.validate_tree_structure():
            sys.exit("ERROR: invalidly linked structure generated by FA: " +
                     self.print_parse(ab, True))
        return ab

    # return true if A(B) is a valid for functional application
    def can_perform_fa(self, i, j, a, b):
        if a is None or b is None:
            # print "A or B is None"  # DEBUG
            return False
        if a.category is None or type(self.lexicon.categories[a.category]) is not list or (
                i - j > 0 and self.lexicon.categories[a.category][1] == 1) or (
                i - j < 0 and self.lexicon.categories[a.category][1] == 0):
            # print "B is left/right when A expects right/left"  # DEBUG
            return False  # B is left/right when A expects right/left
        if not a.is_lambda or not a.is_lambda_instantiation or a.type != b.return_type:
            if a.children is None:  # DEBUG
                sys.exit("ERROR: found lambda with no children: "+str(self.print_parse(a)))
            # print "A is not lambda instantiation or types are mismatched"  # DEBUG
            return False
        if self.lexicon.categories[a.category][2] != b.category:
            # print "B category does not match A expected consumption"  # DEBUG
            return False  # B is not the input category A expects
        if a.parent is None and a.is_lambda and not a.is_lambda_instantiation:
            return True  # the whole tree of A will be replaced with the whole tree of B
        to_traverse = [b]
        b_lambda_context = []
        while len(to_traverse) > 0:
            curr = to_traverse.pop()
            if curr.is_lambda and curr.is_lambda_instantiation:
                b_lambda_context.append(curr.type)
            else:
                break
            to_traverse.extend(b.children)
        # return True if all instances of A lambda appear in lambda contexts identical to what B expects
        return self.lambda_value_replacements_valid(a.children[0], a.lambda_name, [], b,
                                                    b_lambda_context)

    def lambda_value_replacements_valid(self, a, lambda_name, a_lambda_context, b, b_lambda_context):
        # print "checking whether '"+self.print_parse(A)+"' lambda "+str(lambda_name) + \
        #       " instances can be replaced by '"+self.print_parse(B)+"' under contexts " + \
        #       str(A_lambda_context)+","+str(B_lambda_context)  # DEBUG
        if a.is_lambda and a.is_lambda_instantiation:
            extended_context = a_lambda_context[:]
            extended_context.append(a.type)
            return self.lambda_value_replacements_valid(a.children[0], lambda_name, extended_context,
                                                        b, b_lambda_context)
        # this is an instance of A's lambda to be replaced by B
        if a.is_lambda and not a.is_lambda_instantiation and a.lambda_name == lambda_name:
            if a.children is not None and b.children is not None:  # the instance takes arguments
                if (len(a_lambda_context) - len(b_lambda_context) == 1 and
                   b.idx == self.ontology.preds.index('and')):
                    return True  # special 'and' consumption rule (A consumes B 'and' and B takes A's arguments)
                if len(a_lambda_context) != len(b_lambda_context):
                    # print "contexts differ in length"  # DEBUG
                    return False  # the lambda contexts differ in length
                matches = [1 if a_lambda_context[i] == b_lambda_context[i] else 0 for i in
                           range(0, len(a_lambda_context))]
                if sum(matches) != len(a_lambda_context):
                    # print "contexts differ in content"  # DEBUG
                    return False  # the lambda contexts differ in content
                return True
            return True  # ie A, B have no children
        if a.children is None:
            return True
        valid_through_children = True
        for c in a.children:
            valid_through_children = self.lambda_value_replacements_valid(
                c, lambda_name, a_lambda_context, b, b_lambda_context)
            if not valid_through_children:
                break
        return valid_through_children

    # return A,B from A<>B; A<>B must be AND headed and its lambda headers will be distributed to A, B
    def perform_split(self, ab):
        # print "performing Split with '"+self.parser.print_parse(AB,True)+"'" #DEBUG

        curr = ab
        while curr.is_lambda and curr.is_lambda_instantiation:
            curr = curr.children[0]  # first non-lambda must be 'and' predicate
        to_return = []
        for idx in range(0, 2):
            to_return.append(copy.deepcopy(ab))
            curr_t = to_return[-1]
            parent = None
            while curr_t.is_lambda and curr_t.is_lambda_instantiation:
                parent = curr_t
                curr_t = curr_t.children[0]
            if parent is not None:
                parent.children = [copy.deepcopy(curr.children[idx])]
                for c in parent.children:
                    c.parent = parent
            else:
                to_return[-1] = copy.deepcopy(curr.children[idx])
            to_return[-1].set_category(ab.category)
            to_return[-1].set_return_type(self.ontology)

        # print "performed Split with '"+self.parser.print_parse(AB,True)+"' to form '"+ \
        # self.parser.print_parse(to_return[0],True)+"', '"+self.parser.print_parse(to_return[1],True)+"'" #DEBUG
        candidate_pairs = [[to_return[0], to_return[1]], [copy.deepcopy(to_return[1]), copy.deepcopy(to_return[0])]]
        if self.safety:
            for idx in range(0, len(candidate_pairs)):
                for jdx in range(0, len(candidate_pairs[idx])):
                    if not candidate_pairs[idx][jdx].validate_tree_structure():
                        sys.exit("ERROR: invalidly linked structure generated by split: " +
                                 self.print_parse(to_return[-1], True))
        return candidate_pairs

    # return true if AB can be split
    def can_perform_split(self, ab):
        if ab is None:
            return False
        curr = ab
        while curr.is_lambda and curr.is_lambda_instantiation:
            curr = curr.children[0]
        if curr.is_lambda or curr.children is None or curr.idx != self.ontology.preds.index('and'):
            return False
        return True

    # given A1(A2), attempt to determine an A1, A2 that satisfy and return them
    def perform_reverse_fa(self, a):
        consumables = self.lexicon.category_consumes[a.category]
        if len(consumables) == 0:
            return []
        # print "performing reverse FA with '"+self.parser.print_parse(A, True)+"'"  # DEBUG

        # for every predicate p in A of type q, generate candidates:
        # A1 = A with a new outermost lambda of type q, p replaced by an instance of q
        # A2 = p with children stripped
        # if p alone has no unbound variables, generate additional candidates:
        # A1 = A with new outermost lambda of type q return type p, with p and children replaced by q
        # A2 = p with children preserved

        # calculate largest lambda in form
        to_examine = [a]
        deepest_lambda = 0
        while len(to_examine) > 0:
            curr = to_examine.pop()
            if curr.is_lambda_instantiation and curr.lambda_name > deepest_lambda:
                deepest_lambda = curr.lambda_name
            if curr.children is not None:
                to_examine.extend(curr.children)

        candidate_pairs = []
        to_examine = [a]
        while len(to_examine) > 0:
            curr = to_examine.pop()
            if curr.children is not None:
                to_examine.extend(curr.children)
            if curr.is_lambda:
                continue
            pred = curr

            # create A1, A2 with A2 the predicate without children, A1 abstracting A2 instances
            pairs_to_create = [[True, False]]

            # check whether pred is 'and' and children arguments match, in which case additional abstraction
            # is possible
            add_and_abstraction = False
            if pred.idx == self.ontology.preds.index('and') and pred.children is not None:
                arg_children_match = True
                if (pred.children[0].children is not None and pred.children[1].children is not None and
                        len(pred.children[0].children) == len(pred.children[1].children)):
                    ac_to_examine = [[pred.children[0].children[ac_idx],
                                      pred.children[1].children[ac_idx]] for
                                     ac_idx in range(0, len(pred.children[0].children))]
                    while len(ac_to_examine) > 0:
                        c1, c2 = ac_to_examine.pop()
                        if not c1.equal_ignoring_syntax(c2):
                            arg_children_match = False
                            break
                        if c1.children is not None:
                            ac_to_examine.extend([[c1.children[ac_idx], c2.children[ac_idx]]
                                                 for ac_idx in range(0, len(c1.children))])
                else:
                    arg_children_match = False
                if arg_children_match:
                    add_and_abstraction = True

            # create A1, A2 with A2 the predicate and children preserved, A1 abstracting A2 return type
            if pred.children is not None:
                unbound_vars_in_pred = False
                bound_lambda_in_pred = []
                check_bound = [pred]
                while len(check_bound) > 0:
                    curr = check_bound.pop()
                    if curr.is_lambda:
                        if curr.is_lambda_instantiation:
                            bound_lambda_in_pred.append(curr.lambda_name)
                        elif curr.lambda_name not in bound_lambda_in_pred:
                            unbound_vars_in_pred = True
                            break
                    if curr.children is not None:
                        check_bound.extend(curr.children)
                if not unbound_vars_in_pred:
                    pred.set_return_type(self.ontology)
                    pairs_to_create.append([False, add_and_abstraction])

            # create pairs given directives
            for preserve_host_children, and_abstract in pairs_to_create:

                and_abstracts = [False]
                if and_abstract:
                    and_abstracts.append(True)
                for aa in and_abstracts:
                    if preserve_host_children:
                        lambda_type = pred.type
                    elif aa:
                        lambda_type = pred.children[0].type
                    else:
                        lambda_type = pred.return_type
                    a1 = SemanticNode.SemanticNode(
                        None, lambda_type, None, True, lambda_name=deepest_lambda+1, is_lambda_instantiation=True)
                    a1.children = [copy.deepcopy(a)]
                    a1.children[0].parent = a1
                    to_replace = [[a1, 0, a1.children[0]]]
                    # TODO: this procedure replaces all instances of the identified predicate with lambdas
                    # TODO: in principle, however, should have candidates for all possible subsets of replacements
                    # TODO: eg. askperson(ray,ray) -> lambda x.askperson(x,x), lambda x.askperson(ray,x), etc
                    while len(to_replace) > 0:
                        p, c_idx, r = to_replace.pop()
                        if not r.is_lambda and r.idx == pred.idx:
                            lambda_instance = SemanticNode.SemanticNode(
                                p, lambda_type, None, True, lambda_name=deepest_lambda+1,
                                is_lambda_instantiation=False)
                            p.children[c_idx].copy_attributes(
                                lambda_instance, preserve_parent=True, preserve_children=preserve_host_children)
                            if aa:
                                p.children[c_idx].children = copy.deepcopy(pred.children[0].children)
                                for c in p.children[c_idx].children:
                                    c.parent = p.children[c_idx]
                        if r.children is not None:
                            to_replace.extend([[r, idx, r.children[idx]] for idx in range(0, len(r.children))])
                    # print "A1 before renumeration: "+self.parser.print_parse(A1, True)  # DEBUG
                    a1.renumerate_lambdas([])
                    a1.set_return_type(self.ontology)
                    a2 = copy.deepcopy(pred)
                    if preserve_host_children:
                        a2.children = None
                    if aa:
                        for c in a2.children:
                            c.children = None
                            c.set_return_type(self.ontology)
                            c.parent = a2
                        input_type = [a2.children[0].return_type, a2.children[0].return_type]
                        if input_type not in self.ontology.types:
                            self.ontology.types.append(input_type)
                        full_type = [a2.children[0].return_type, self.ontology.types.index(input_type)]
                        if full_type not in self.ontology.types:
                            self.ontology.types.append(full_type)
                        a2.type = self.ontology.types.index(full_type)
                        a2.set_return_type(self.ontology)
                    for d, cat in consumables:
                        a1_with_cat = copy.deepcopy(a1)
                        a1_with_cat.set_category(self.lexicon.categories.index([a.category, d, cat]))
                        a2_with_cat = copy.deepcopy(a2)
                        a2_with_cat.set_category(cat)
                        candidate_pairs.append([a1_with_cat, d, a2_with_cat])
                        # print "produced: "+self.parser.print_parse(A1_with_cat, True)+" consuming " + \
                        # self.parser.print_parse(A2_with_cat, True)+" in dir "+str(d)+" with params " + \
                        # ",".join([str(lambda_type), str(preserve_host_children), str(aa)])  # DEBUG
                        if self.safety and not a1_with_cat.validate_tree_structure():
                            sys.exit("ERROR: invalidly linked structure generated by reverse FA: " +
                                     self.print_parse(a1_with_cat, True) +
                                     "with params "+",".join([str(lambda_type), str(preserve_host_children), str(aa)]))
                        if self.safety and not a2_with_cat.validate_tree_structure():
                            sys.exit("ERROR: invalidly linked structure generated by reverse FA: " +
                                     self.print_parse(a2_with_cat, True) +
                                     "with params "+",".join([str(lambda_type), str(preserve_host_children), str(aa)]))

        return candidate_pairs

    # given a string, return the set of possible tokenizations of that string given lexical entries
    def tokenize(self, s):
        str_parts = s.split()
        tokenizations = []
        max_span_length = [0 for _ in range(0, len(str_parts))]  # how long each span can be in order of discovery
        while max(max_span_length) <= self.max_multiword_expression:
            if sum(max_span_length) == len(str_parts):
                i = 0
                sp = 0
                tokenization = []
                to_cover = range(0, len(str_parts))
                while i < len(str_parts):
                    span_len = 0
                    unk = True if str_parts[i] not in self.lexicon.surface_forms else False
                    while (span_len < max_span_length[sp] and i+span_len < len(str_parts)
                           and (" ".join(str_parts[i:i+span_len+1]) in self.lexicon.surface_forms or
                                (unk and str_parts[i+span_len] not in self.lexicon.surface_forms))):
                        span_len += 1
                    if span_len == max_span_length[sp]:
                        tokenization.append(' '.join(str_parts[i:i+span_len]))
                        for j in range(i, i+span_len):
                            del to_cover[to_cover.index(j)]
                        i += span_len
                        sp += 1
                    else:
                        break  # no matching span at required position
                    if sp == len(max_span_length):
                        break  # out of combinations
                if len(to_cover) == 0:
                    tokenizations.append(tokenization)
            max_span_length[0] += 1
            for j in range(0, len(str_parts)-1):
                if max_span_length[j] > self.max_multiword_expression:
                    max_span_length[j] = 1
                    max_span_length[j+1] += 1

        return tokenizations
