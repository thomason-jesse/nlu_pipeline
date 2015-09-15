__author__ = 'jesse'

import copy
import sys
import SemanticNode


class Generator:
    def __init__(self, ont, lex, learner, beam_width=10):
        self.ontology = ont
        self.lexicon = lex
        self.learner = learner
        self.beam_width = beam_width
        self.bad_nodes = []  # nodes for which we know there are no tokens and no expansions
        self.known_terminals = []  # nodes that can't be further expanded

    # flush bad nodes structure whenever lexicon additions happen
    def flush_bad_nodes(self):
        self.bad_nodes = []

    # print a SemanticNode as a string using the known ontology
    # DEBUG use only
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

    # given semantic form, return a list of up to k [tokens,semantic parse tree,score] pairs;
    # less than k returned if fewer than k explanatory tokens could be found
    def reverse_parse_semantic_form(self, root, k=None, n=1):
        if k is None:
            k = self.beam_width

        full_parses = []
        partial_parses = [[[root], [None, False]]]  # set of partials, parse bracketing
        while len(full_parses) < n and len(partial_parses) > 0:
            # print "num partial parses: "+str(len(partial_parses))  # DEBUG
            # print "num full parses: "+str(len(full_parses))  # DEBUG

            # find best scoring partial and expand it
            scores = [self.learner.scoreParse([], pp) for pp in partial_parses]
            max_score_idx = scores.index(max(scores))
            # print "max scoring parse "+str([
            #     self.print_parse(p) for p in partial_parses[max_score_idx][0]])  # DEBUG

            # if partial token sequence matches its size, the parse is finished
            possible_assignments = self.assign_tokens_to_partial(
                partial_parses[max_score_idx][0], partial_parses[max_score_idx][1])
            for pa in possible_assignments:
                tpa = self.extract_token_sequence_from_bracketing(pa)
                if len(tpa) == len(partial_parses[max_score_idx][0]):
                    full_parses.append([partial_parses[max_score_idx][0], pa])
            else:
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

        # score full parses and return n best in order with scores
        full_parse_scores = [self.learner.scoreParse(
            self.extract_token_sequence_from_bracketing(fp[1]), fp) for fp in full_parses]
        k_best = []
        while len(k_best) < n and len(full_parses) > 0:
            max_score_idx = full_parse_scores.index(max(full_parse_scores))
            k_best.append([
                self.extract_token_sequence_from_bracketing(full_parses[max_score_idx][1]),
                full_parses[max_score_idx][1], full_parse_scores[max_score_idx]])
            del full_parses[max_score_idx]
            del full_parse_scores[max_score_idx]
        return k_best

    # given partial parse
    # decompose via function application (forwards and backwards), type-raising, and merge operations
    def expand_partial_parse(self, p):
        partial = p[0]
        tree = p[1]
        # print "splitting parse "+str([self.print_parse(p) for p in partial])
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
                    bad[split_idx] = True
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
            # print "expanded partial: "+str([self.print_parse(p) for p in expanded_partial])  # DEBUG
            expanded_partials.append([expanded_partial, expanded_tree])
        # print "num expanded partials: "+str(len(expanded_partials))  # DEBUG
        return expanded_partials

    # return N/N : lambda x.(pred(x)) lowered to DESC : pred
    def perform_adjectival_type_lower(self, A):
        # print "performing adjectival lower with '"+self.print_parse(A,True)+"'" #DEBUG
        pred = copy.deepcopy(A.children[0])
        pred.children = None
        pred.category = self.lexicon.categories.index('DESC')  # change from DESC return type
        pred.type = self.ontology.types([
            self.ontology.types.index('e'), self.ontology.types.index('t')])
        pred.set_return_type(self.ontology)
        # print "performed adjectival lower with '"+self.print_parse(A,True)+"' to form '"+self.print_parse(pred,True)+"'" #DEBUG
        return pred

    # return true if A is a candidate for type-raising
    def can_perform_type_lowering(self, A):
        if 'DESC' not in self.lexicon.categories or A is None:
            return False
        # check for N/N : lambda x.(pred(x)) -> DESC : pred lower
        if (A.is_lambda_instantiation and A.children is not None and
                A.children[0].category == self.lexicon.categories.index([
                self.lexicon.categories.index('N'), 1, self.lexicon.categories.index('N')])
                and A.children[0].children is not None and
                A.children[0].children[0].lambda_name == A.lambda_name):
            return "N/N->DESC"
        return False

    # return A,B from A<>B; A<>B must be AND headed and its lambda headers will be distributed to A, B
    def perform_split(self, AB):
        # print "performing Split with '"+self.print_parse(AB,True)+"'" #DEBUG

        to_return = []
        for idx in range(0, 2):
            to_return.append(copy.deepcopy(AB))
            curr = to_return[idx]
            while curr.is_lambda and curr.is_lambda_instantiation:
                curr = curr.children[0]
            # first non-lambda must be 'and' predicate
            curr.parent.children[0] = curr.children[idx]  # replace 'and' with child, preserving lambda headers
            curr.parent.children[0].set_return_type(self.ontology)

        # print "performed Split with '"+self.print_parse(AB,True)+"' to form '"+self.print_parse(to_return[0],True)+"', '"+self.print_parse(to_return[1],True)+"'" #DEBUG
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
        # print "performing reverse FA with '"+self.print_parse(A,True)+"'"

        # TODO: if A is 'and', A1 can be 'and' with A1 applied to its children to form A

        # for every predicate p in A of type q, generate candidates:
        # A1 = A with a new outermost lambda of type q, p replaced by an instance of q
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
            pred = curr.idx
            pred_type = curr.type
            A1 = SemanticNode.SemanticNode(
                None, pred_type, None, True, lambda_name=deepest_lambda+1, is_lambda_instantiation=True)
            A1.children = [copy.deepcopy(A)]
            to_replace = [[A1, 0, A1.children[0]]]
            # TODO: this procedure replaces all instances of the identified predicate with lambdas
            # TODO: in principle, however, should have candidates for all possible subsets of replacements
            # TODO: eg. askperson(ray,ray) -> lambda x.askperson(x,x), lambda x.askperson(ray,x), etc
            # TODO: additionally, allow abstracting predicate with children, eg speak_t(person(ray)) ->
            # TODO: lambda x:t.(speak_t(x)) with person(ray)
            while len(to_replace) > 0:
                p, c_idx, r = to_replace.pop()
                if not r.is_lambda and r.idx == pred:
                    lambda_instance = SemanticNode.SemanticNode(
                        p, pred_type, None, True, lambda_name=deepest_lambda+1, is_lambda_instantiation=False)
                    p.children[c_idx].copy_attributes(lambda_instance, preserve_parent=True, preserve_children=True)
                if r.children is not None:
                    to_replace.extend([[r, idx, r.children[idx]] for idx in range(0, len(r.children))])
            self.renumerate_lambdas(A1, [])
            A2 = copy.deepcopy(curr)
            A2.children = None
            candidate_pairs.append([A1, A2])

        # print "performed reverse FA with '"+self.print_parse(A,True)+"', producing:"  # DEBUG
        # for candidates in candidate_pairs:  # DEBUG
        #     print self.print_parse(candidates[0])+" consuming "+self.print_parse(candidates[1])  # DEBUG

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
