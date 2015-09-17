__author__ = 'jesse'

import copy
import sys
import math
import SemanticNode


class Generator:
    def __init__(self, ont, lex, learner, parser, beam_width=10):
        self.ontology = ont
        self.lexicon = lex
        self.learner = learner
        self.parser = parser
        self.beam_width = beam_width
        self.bad_nodes = []  # nodes for which we know there are no tokens and no expansions
        self.known_terminals = []  # nodes that can't be further expanded

    # flush bad nodes structure whenever lexicon additions happen
    def flush_bad_nodes(self):
        self.bad_nodes = []

    # given bracketing, return sequence of tokens
    def extract_token_sequence_from_bracketing(self, b, preserve_False=False):
        t = []
        if type(b[0]) is str:
            t.append(b[0])  # tokens which can head merges
        if type(b[1]) is list:
            for i in range(0, len(b[1])):
                if type(b[1][i]) is list:
                    t.extend(self.extract_token_sequence_from_bracketing(b[1][i], preserve_False=preserve_False))
                elif type(b[1][i]) is str:
                    t.append(b[1][i])
                elif not b[1][i] and preserve_False:
                    t.append(b[1][i])
        elif type(b[1]) is str:
            t.append(b[1])
        elif not b[1] and preserve_False:
            t.append(b[1])
        return t

    # given partial, try to assign tokens to leaves and update bracketing accordingly
    def assign_tokens_to_partial(self, r, b):

        # calculate candidate surface forms for each current partial
        candidate_tokens = []
        for p in r:
            candidate_tokens.append(self.get_tokens_for_semantic_node(p))

        # build list of each possible combination and return
        return self.add_token_assignments_to_bracketing(b, 0, candidate_tokens)

    # get candidate tokens for a given semantic node
    def get_tokens_for_semantic_node(self, n):
        tokens = []
        for sem_idx in range(0, len(self.lexicon.semantic_forms)):
            if n.equal_ignoring_syntax(self.lexicon.semantic_forms[sem_idx]):
                tokens.extend([
                    [sem_idx, self.lexicon.surface_forms[i]]
                    for i in self.lexicon.reverse_entries[sem_idx]])
                break
        return tokens

    def add_token_assignments_to_bracketing(self, b, idx, candidate_tokens):
        all_assignments = []
        for form, token in candidate_tokens[idx]:
            b_with_token = copy.deepcopy(b)
            success = self.replace_bracket_leaf_with_token_assignment(b_with_token, form, token)
            if not success:
                sys.exit("ERROR: failed to make full assignment for "+str(b)+" with tokens "+str(candidate_tokens))
            if idx < len(candidate_tokens)-1:
                all_assignments.extend(self.add_token_assignments_to_bracketing(
                    b_with_token, idx+1, candidate_tokens))
            else:
                all_assignments.append(b_with_token)
        return all_assignments

    def replace_bracket_leaf_with_token_assignment(self, b, f, t):
        made_assignment = False
        if type(b[1]) is list:
            for ci in range(0, len(b[1])):
                if type(b[1][ci]) is list:
                    made_assignment = self.replace_bracket_leaf_with_token_assignment(b[1][ci], f, t)
                    if made_assignment:
                        break
                elif not b[1][ci]:
                    b[1][ci] = [f, t]
                    made_assignment = True
                    break
        elif not b[1]:
            b[1] = [f, t]
            made_assignment = True
        return made_assignment

    def count_preds_in_semantic_form(self, r):
        n = 0
        if not r.is_lambda:
            n += 1
        if r.children is not None:
            for c in r.children:
                n += self.count_preds_in_semantic_form(c)
        return n

    # given semantic form, return a list of up to n [tokens,semantic parse tree,score] pairs;
    # fewer than k returned if fewer than k explanatory tokens could be found
    # this function first reverse-parses the given root to come up with up to k sequences, then
    # runs the parser on those sequences to generate and return up to n best according to parse scores
    def reverse_parse_semantic_form(self, root, k=None, n=1):
        if k is None:
            k = self.beam_width
        candidate_sequences = self.get_token_sequences_for_semantic_form(root, n=k)
        trees = []
        scores = []
        for tokens in candidate_sequences:
            tree = None
            score = None
            parses = self.parser.parse_tokens(tokens)
            # parses ordered by decreasing score, so first to match is best
            for parse in parses:
                if parse[0].equal_ignoring_syntax(root):
                    tree = parse[1]
                    score = parse[2]
                    break
            trees.append(tree)
            scores.append(score)
        n_best = []
        while len(n_best) < n and len(candidate_sequences) > 0:
            max_score_idx = scores.index(max(scores))
            if trees[max_score_idx] is not None:
                n_best.append([candidate_sequences[max_score_idx], trees[max_score_idx], scores[max_score_idx]])
            else:
                print "WARNING: sequence " + str(candidate_sequences[max_score_idx]) +\
                      " was generated but does not parse into expected root"
            del candidate_sequences[max_score_idx]
            del trees[max_score_idx]
            del scores[max_score_idx]
        return n_best

    # given semantic form, return a list of up to n token sequences;
    # fewer than n returned if fewer than n explanatory tokens could be found
    def get_token_sequences_for_semantic_form(self, root, k=None, n=1):
        if k is None:
            k = self.beam_width
        num_preds = self.count_preds_in_semantic_form(root)
        search_space_limit = max([100, int(math.factorial(num_preds))])

        full_parses = []
        partial_parses = [[[root], [None, False]]]  # set of partials, parse bracketing
        while len(full_parses) < n and len(partial_parses) > 0 and len(partial_parses) < search_space_limit:
            # print "num partial parses: "+str(len(partial_parses))  # DEBUG
            # print "num full parses: "+str(len(full_parses))  # DEBUG

            # find best scoring partial and expand it
            scores = [self.score_top_down_partial(pp[0]) for pp in partial_parses]  # use in-house heuristic
            max_score_idx = scores.index(max(scores))
            # print "max scoring parse "+str([
            #     self.print_parse(p) for p in partial_parses[max_score_idx][0]]) +\
            #     ", " + str(scores[max_score_idx])  # DEBUG

            # if partial token sequence matches its size, the parse is finished
            possible_assignments = self.assign_tokens_to_partial(
                partial_parses[max_score_idx][0], partial_parses[max_score_idx][1])
            was_full_parse = False
            for pa in possible_assignments:
                tpa = self.extract_token_sequence_from_bracketing(pa, preserve_False=True)
                if False not in tpa and len(tpa) == len(partial_parses[max_score_idx][0]):
                    if len(tpa) == 1 and pa[0] is None and type(pa[1]) is list:
                        # this is the single-word case where our compositionality assumption is violated
                        # so we need to make the bracketing the single idx,word pair
                        pa = pa[1]
                    candidate = [partial_parses[max_score_idx][0], pa]
                    if candidate not in full_parses:
                        full_parses.append(candidate)
                    was_full_parse = True
            if not was_full_parse:
                # else, expand the parse
                new_partials = [[cpp, cpsp] for [cpp, cpsp] in
                                self.expand_partial_parse(partial_parses[max_score_idx])]
                # score new parses and add to partials / full where appropriate
                new_partials_scores = [self.learner.scoreParse([], npp) for npp in new_partials]
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

        if len(partial_parses) >= search_space_limit:
            print "WARNING: potential parses exceeded search space limit "+str(search_space_limit)

        # score full parses and return n best in order according to in-house scoring heuristic
        full_parse_scores = [self.score_top_down_partial(fp[0]) for fp in full_parses]
        n_best = []
        while len(n_best) < n and len(full_parses) > 0:
            max_score_idx = full_parse_scores.index(max(full_parse_scores))
            t = self.extract_token_sequence_from_bracketing(full_parses[max_score_idx][1])
            if t not in n_best:
                n_best.append(t)
            del full_parses[max_score_idx]
            del full_parse_scores[max_score_idx]
        return n_best

    # in-house heuristic to guide search; rewards parses for node brevity and a high ratio of assignable leaves
    def score_top_down_partial(self, partial):
        token_score = 0
        for p in partial:
            num_t = len(self.get_tokens_for_semantic_node(p))
            if num_t > 0:
                token_score += 1/float(num_t)
        return token_score/float(len(partial)) - math.sqrt(len(partial))

    # given partial parse
    # decompose via function application (forwards and backwards), type-raising, and merge operations
    def expand_partial_parse(self, p):
        partial = p[0]
        tree = p[1]
        # print "splitting parse "+str([self.parser.print_parse(p) for p in partial])
        possible_splits = []
        for i in range(0, len(partial)):
            if partial[i] in self.known_terminals or partial[i] in self.bad_nodes:
                continue
            orig_num_splits = len(possible_splits)

            # try reverse FA
            split_subtrees = self.perform_reverse_fa(partial[i])
            possible_splits.extend([[i, ss[0], ss[1], None] for ss in split_subtrees])
            # try merge split
            if self.can_perform_split(partial[i]):
                ss = self.perform_split(partial[i])
                possible_splits.append([i, ss[0], ss[1], None])
            # try type lowering on i
            if self.can_perform_type_lowering(partial[i]) == "N/N->DESC":
                ls = self.perform_adjectival_type_lower(partial[i])
                possible_splits.append([i, ls, False, None])

            # check whether this is a bad node
            if orig_num_splits == len(possible_splits):  # cannot be further expanded
                self.known_terminals.append(partial[i])
                if len(self.get_tokens_for_semantic_node(partial[i])) == 0:  # no token assignments
                    if partial[i] not in self.bad_nodes:
                        self.bad_nodes.append(partial[i])

        expanded_partials = []
        for split in possible_splits:

            # check whether any nodes in new partials are bad, and don't use those partials if so
            # add source of these nodes to bad nodes list if they are both bad
            bad = [False, False]
            for split_idx in range(1, 3):
                if split[split_idx] in self.bad_nodes:
                    bad[split_idx-1] = True
            if True in bad:
                if bad[0] and bad[1]:
                    if partial[split[0]] not in self.bad_nodes:
                        self.bad_nodes.append(partial[split[0]])
                continue

            # perform splitting and add to return structure
            expanded_partial = partial[:split[0]]
            if split[2] is not False:
                expanded_partial.extend([split[1], split[2]])
            else:
                expanded_partial.append(split[1])
            expanded_partial.extend(partial[split[0] + 1:])
            expanded_tree = copy.deepcopy(tree)
            to_expand = [expanded_tree]
            leaves_seen = 0
            while leaves_seen < split[0]+1:
                curr = to_expand.pop()
                if type(curr[1]) is list:
                    for ci in range(0, len(curr[1])):
                        if type(curr[1][ci]) is list:
                            to_expand.append(curr[1][ci])
                        elif not curr[1][ci]:
                            if leaves_seen == split[0]:
                                curr[1][ci] = [split[3], [False, False]]
                            leaves_seen += 1
                elif not curr[1]:
                    if leaves_seen == split[0]:
                        curr[0] = split[3]
                        curr[1] = [False, False]
                    leaves_seen += 1
            # print "expanded partial: "+str([self.parser.print_parse(p) for p in expanded_partial])  # DEBUG
            expanded_partials.append([expanded_partial, expanded_tree])
        # print "num expanded partials: "+str(len(expanded_partials))  # DEBUG
        # empty = raw_input()
        return expanded_partials

    # return N/N : lambda x.(pred(x)) lowered to DESC : pred
    def perform_adjectival_type_lower(self, A):
        # print "performing adjectival lower with '"+self.parser.print_parse(A,True)+"'" #DEBUG
        pred = copy.deepcopy(A.children[0])
        pred.children = None
        pred.category = self.lexicon.categories.index('DESC')  # change from DESC return type
        pred.type = self.ontology.types.index([
            self.ontology.types.index('e'), self.ontology.types.index('t')])
        pred.set_return_type(self.ontology)
        # print "performed adjectival lower with '"+self.parser.print_parse(A,True)+"' to form '"+self.pasrer.print_parse(pred,True)+"'" #DEBUG
        return pred

    # return true if A is a candidate for type-raising
    def can_perform_type_lowering(self, A):
        if 'DESC' not in self.lexicon.categories or A is None:
            return False
        # check for N/N : lambda x.(pred(x)) -> DESC : pred lower
        # because we don't track categories, must see whether DESC : pred is a leaf-level entry in lexicon
        if (A.is_lambda_instantiation and A.children is not None and len(A.children) == 1 and
                A.children[0].idx is not None and A.children[0].children is not None and
                len(A.children[0].children) == 1 and
                A.children[0].children[0].lambda_name == A.lambda_name):
            pred_idx = A.children[0].idx
            for sur_idx in self.lexicon.pred_to_surface[pred_idx]:
                for sem_idx in self.lexicon.entries[sur_idx]:
                    sem = self.lexicon.semantic_forms[sem_idx]
                    if sem.idx == pred_idx and sem.category == self.lexicon.categories.index('DESC'):
                        return "N/N->DESC"
        return False

    # return A,B from A<>B; A<>B must be AND headed and its lambda headers will be distributed to A, B
    def perform_split(self, AB):
        # print "performing Split with '"+self.parser.print_parse(AB,True)+"'" #DEBUG

        curr = AB
        while curr.is_lambda and curr.is_lambda_instantiation:
            curr = curr.children[0]  # first non-lambda must be 'and' predicate
        to_return = []
        for idx in range(0, 2):
            to_return.append(copy.deepcopy(AB))
            curr_t = to_return[-1]
            parent = None
            while curr_t.is_lambda and curr_t.is_lambda_instantiation:
                parent = curr_t
                curr_t = curr_t.children[0]
            if parent is not None:
                parent.children = [copy.deepcopy(curr.children[idx])]
            else:
                to_return[-1] = copy.deepcopy(curr.children[idx])
            to_return[-1].set_return_type(self.ontology)

        # print "performed Split with '"+self.parser.print_parse(AB,True)+"' to form '"+self.parser.print_parse(to_return[0],True)+"', '"+self.parser.print_parse(to_return[1],True)+"'" #DEBUG
        return to_return

    # return true if AB can be split
    def can_perform_split(self, AB):
        if AB is None:
            return False
        curr = AB
        while curr.is_lambda and curr.is_lambda_instantiation:
            curr = curr.children[0]
        if curr.is_lambda or curr.children is None or curr.idx != self.ontology.preds.index('and'):
            return False
        return True

    # given A1(A2), attempt to determine an A1, A2 that satisfy and return them
    def perform_reverse_fa(self, A):
        # print "performing reverse FA with '"+self.parser.print_parse(A,True)+"'"

        # for every predicate p in A of type q, generate candidates:
        # A1 = A with a new outermost lambda of type q, p replaced by an instance of q
        # A2 = p with children stripped
        # if p alone has no unbound variables, generate additional candidates:
        # A1 = A with new outermost lambda of type q return type p, with p and children replaced by q
        # A2 = p with children preserved

        candidate_pairs = []
        to_examine = [A]
        deepest_lambda = 0
        while len(to_examine) > 0:
            curr = to_examine.pop()
            if curr.is_lambda_instantiation:
                deepest_lambda = curr.lambda_name
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
                    A1 = SemanticNode.SemanticNode(
                        None, lambda_type, None, True, lambda_name=deepest_lambda+1, is_lambda_instantiation=True)
                    A1.children = [copy.deepcopy(A)]
                    to_replace = [[A1, 0, A1.children[0]]]
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
                        if r.children is not None:
                            to_replace.extend([[r, idx, r.children[idx]] for idx in range(0, len(r.children))])
                    self.renumerate_lambdas(A1, [])
                    A2 = copy.deepcopy(pred)
                    if preserve_host_children:
                        A2.children = None
                    if aa:
                        for c in A2.children:
                            c.children = None
                            c.set_return_type(self.ontology)
                        input_type = [A2.children[0].return_type, A2.children[0].return_type]
                        if input_type not in self.ontology.types:
                            self.ontology.types.append(input_type)
                        full_type = [A2.children[0].return_type, self.ontology.types.index(input_type)]
                        if full_type not in self.ontology.types:
                            self.ontology.types.append(full_type)
                        A2.type = self.ontology.types.index(full_type)
                        A2.set_return_type(self.ontology)
                    candidate_pairs.append([A1, A2])
                    # print "produced: "+self.parser.print_parse(A1)+" consuming "+self.parser.print_parse(A2)+" with params "+",".join([str(lambda_type),str(preserve_host_children),str(aa)])  # DEBUG

        return candidate_pairs

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
