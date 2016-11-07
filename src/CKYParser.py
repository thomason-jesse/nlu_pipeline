__author__ = 'jesse'

import random
import math
import operator
import copy
import sys
import ParseNode
import SemanticNode

from IPython import embed  # DEBUG

neg_inf = float('-inf')
random.seed(4)

class Parameters:
    def __init__(self, ont, lex, use_language_model=False, lexicon_weight=1.0):
        self.ontology = ont
        self.lexicon = lex
        self.use_language_model = use_language_model

        # get initial count data structures
        self._token_given_token_counts = {}
        self._CCG_given_token_counts = self.init_ccg_given_token(lexicon_weight)
        self._CCG_production_counts = self.init_ccg_production(lexicon_weight)
        self._CCG_root_counts = {}
        self._lexicon_entry_given_token_counts = self.init_lexicon_entry(lexicon_weight)
        self._semantic_counts = {}

        # print "_CCG_given_token_counts: "+str(self._CCG_given_token_counts)  # DEBUG
        # print "_CCG_production_counts: "+str(self._CCG_production_counts)  # DEBUG
        # print "_lexicon_entry_counts: "+str(self._lexicon_entry_given_token_counts)  # DEBUG
        # print "_semantic_counts: "+str(self._semantic_counts)  # DEBUG

        # calculate probability tables from counts
        # note that this needs to happen every time counts are updated
        self.token_given_token = {}
        self.CCG_given_token = {}
        self.CCG_production = {}
        self.CCG_root = {}
        self.lexicon_entry_given_token = {}
        self.semantic = {}

        # update probabilities given new counts
        self.update_probabilities()

        # print "CCG_given_token: "+str(self.CCG_given_token)  # DEBUG
        # print "CCG_production: "+str(self.CCG_production)  # DEBUG
        # print "lexicon_entry: "+str(self.lexicon_entry_given_token)  # DEBUG
        # print "semantic: "+str(self.semantic)  # DEBUG

    # update the probability tables given counts
    def update_probabilities(self):

        language_model_surface_forms = range(-1, len(self.lexicon.surface_forms))
        if self.use_language_model:
            # get log probabilities for token_given_token ( P(sf|sf), P(sf=s|S)=1 )
            # use special indices -2 for start, -3 for end, -1 for unknown token
            language_model_surface_forms.extend([-2, -3])
            for sf_idx in language_model_surface_forms:
                for next_idx in language_model_surface_forms:
                    if (sf_idx, next_idx) not in self._token_given_token_counts:
                        self._token_given_token_counts[(sf_idx, next_idx)] = 0
                nums = [self._token_given_token_counts[(sf_idx, next_idx)]
                        for next_idx in language_model_surface_forms]
                num_min = 0
                mass = 0
                if len(nums) > 0:
                    num_min = float(min(nums))
                    mass = float(sum(nums)) - num_min*len(nums)
                for next_idx in language_model_surface_forms:
                    key = (sf_idx, next_idx)
                    self.token_given_token[key] = math.log((1+self._token_given_token_counts[key]-num_min) / mass) \
                        if mass > 0 else math.log(1.0 / len(nums))

        # get log probabilities for CCG_root p(root)
        self._CCG_root_counts[-1] = 0  # hallucinate unseen roots score
        nums = [self._CCG_root_counts[cat_idx] for cat_idx in self._CCG_root_counts]
        num_min = 0
        mass = 0
        if len(nums) > 0:
            num_min = float(min(nums))
            mass = float(sum(nums)) - num_min*len(nums)
        for key in self._CCG_root_counts:
            self.CCG_root[key] = (math.log((1+self._CCG_root_counts[key]-num_min) / mass)
                                  if mass > 0 else math.log(1.0 / len(nums)))

        for cat_idx in range(0, len(self.lexicon.categories)):

            # get log probabilities for CCG_given_token ( P(ccg|surface), P(ccg=c|S)=1 )
            self._CCG_given_token_counts[(cat_idx, -1)] = 0  # hallucinate missing entries
            nums = [self._CCG_given_token_counts[(cat_idx, sf_idx)]
                    for sf_idx in range(-1, len(self.lexicon.surface_forms))
                    if (cat_idx, sf_idx) in self._CCG_given_token_counts]
            num_min = 0
            mass = 0
            if len(nums) > 0:
                num_min = float(min(nums))
                mass = float(sum(nums)) - num_min*len(nums)
            for sf_idx in range(-1, len(self.lexicon.surface_forms)):
                key = (cat_idx, sf_idx)
                if key in self._CCG_given_token_counts:
                    self.CCG_given_token[key] = math.log((1+self._CCG_given_token_counts[key]-num_min) / mass) \
                        if mass > 0 else math.log(1.0 / len(nums))

                # get log probabilities for lexicon_entry_given_token
                # ( P(sem|surface), P(sem=s|S)=1 )
                for sem_idx in range(0, len(self.lexicon.semantic_forms)):
                    if (sem_idx, sf_idx) not in self._lexicon_entry_given_token_counts:
                        self._lexicon_entry_given_token_counts[(sem_idx, sf_idx)] = 0  # hallucinate missings
                entry_nums = [self._lexicon_entry_given_token_counts[(sem_idx, sf_idx)]
                              for sem_idx in (self.lexicon.entries[sf_idx]
                              if sf_idx > -1 else range(0, len(self.lexicon.semantic_forms)))]
                entry_num_min = 0
                entry_mass = 0
                if len(entry_nums) > 0:
                    entry_num_min = float(min(entry_nums))
                    entry_mass = float(sum(entry_nums)) - entry_num_min*len(entry_nums)
                for sem_idx in (self.lexicon.entries[sf_idx]
                                if sf_idx > -1 else range(0, len(self.lexicon.semantic_forms))):
                    key = (sem_idx, sf_idx)
                    self.lexicon_entry_given_token[key] = math.log(
                        (1+self._lexicon_entry_given_token_counts[key]-entry_num_min) / entry_mass) \
                        if entry_mass > 0 else math.log(1.0 / len(entry_nums))

            # get log probabilities for CCG_production ( P(ccg|(leftcgg, rightccg)), P(ccg=c|LR)=1 )
            nums = [self._CCG_production_counts[(cat_idx, l_idx, r_idx)]
                    for l_idx in range(0, len(self.lexicon.categories))
                    for r_idx in range(0, len(self.lexicon.categories))
                    if (cat_idx, l_idx, r_idx) in self._CCG_production_counts]
            if len(nums) == 0:
                continue
            num_min = float(min(nums))
            mass = float(sum(nums)) - num_min*len(nums)
            for l_idx in range(0, len(self.lexicon.categories)):
                for r_idx in range(0, len(self.lexicon.categories)):
                    key = (cat_idx, l_idx, r_idx)
                    if key in self._CCG_production_counts:
                        self.CCG_production[key] = math.log((1+self._CCG_production_counts[key]-num_min) / mass) \
                            if mass > 0 else math.log(1.0 / len(nums))

        # get log probabilities for semantic_args ( P(arg|(pred,pos)), P(arg=a|(P, PS))=1
        for arg_idx in range(0, len(self.ontology.preds)):
            nums = [self._semantic_counts[(pred_idx, arg_idx, pos)]
                    for pred_idx in range(0, len(self.ontology.preds))
                    for pos in range(0, self.ontology.num_args[pred_idx])
                    if (pred_idx, arg_idx, pos) in self._semantic_counts]
            if len(nums) == 0:
                continue
            num_min = float(min(nums))
            mass = float(sum(nums)) - num_min*len(nums)
            for pred_idx in range(0, len(self.ontology.preds)):
                for pos in range(0, self.ontology.num_args[pred_idx]):
                    key = (pred_idx, arg_idx, pos)
                    if key in self._semantic_counts:
                        self.semantic[key] = math.log((1+self._semantic_counts[key]-num_min) / mass) \
                            if mass > 0 else math.log(1.0 / len(nums))

    # indexed by (categories idx, surface forms idx), value parameter weight
    def init_ccg_given_token(self, lexicon_weight):
        return {(self.lexicon.semantic_forms[sem_idx].category, sf_idx): lexicon_weight
                for sf_idx in range(0, len(self.lexicon.entries))
                for sem_idx in self.lexicon.entries[sf_idx]}

    # indexed by categories idxs (production, left, right), value parameter weight
    def init_ccg_production(self, lexicon_weight):
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
                ccg_production[(cat_idx, l, r)] = lexicon_weight

            # add production rules of form X -> X X for X<>X=X (merge)
            ccg_production[(cat_idx, cat_idx, cat_idx)] = lexicon_weight

        return ccg_production

    # indexed by (surface forms idx, semantic forms idx), value parameter weight
    def init_lexicon_entry(self, lexicon_weight):
        return {(sem_idx, sf_idx): lexicon_weight
                for sf_idx in range(0, len(self.lexicon.entries))
                for sem_idx in self.lexicon.entries[sf_idx]}

    # takes in a parse node and returns its log probability
    def get_semantic_score(self, n):

        counts = self.count_semantics(n)
        score = 0.0
        for key in counts:
            if key in self.semantic:
                for _ in range(0, counts[key]):
                    score += self.semantic[key]
        return score

    # take in ParseNode y and calculate bigram token counts as dictionary
    def count_token_bigrams(self, y):
        t = [l.surface_form for l in y.get_leaves()]
        for t_idx in range(0, len(t)):
            if type(t[t_idx]) is str:
                t[t_idx] = self.lexicon.surface_forms.index(t[t_idx])
            if t[t_idx] is None:
                raise RuntimeError("Leaf parse node has None surface form "+str(y))
        b = {}
        t.insert(0, -2)  # beginning of sentence token
        t.append(-3)  # end of sentence token
        for t_idx in range(0, len(t)-1):
            key = (t[t_idx], t[t_idx+1])
            if key not in b:
                b[key] = 0
            b[key] += 1
        return b

    # takes in a parse or semantic node and returns the counts of its (pred, arg, pos) entries
    def count_semantics(self, sn):

        # convert passed ParseNodes to SemanticNode member
        try:
            sn = sn.node
        except AttributeError:
            pass

        counts = {}
        if sn.children is not None:
            pred = sn.idx
            if sn.is_lambda:
                pred = "lambda_inst" if sn.is_lambda_instantiation else "lambda"
            for pos in range(0, len(sn.children)):
                arg = sn.children[pos].idx
                if sn.children[pos].is_lambda:
                    arg = "lambda_inst" if sn.children[pos].is_lambda_instantiation else "lambda"
                key = (pred, arg, pos)
                if key not in counts:
                    counts[key] = 0
                counts[key] += 1

            for c in sn.children:
                child_counts = self.count_semantics(c)
                for ckey in child_counts:
                    if ckey not in counts:
                        counts[ckey] = 0
                    counts[ckey] += child_counts[ckey]

        return counts

    # take in examples t=(x,y,z) for x an expression, y an incorrect semantic form, z a correct semantic form
    def update_learned_parameters(self, t):

        # update counts given new training data
        for x, y, z, y_lex, z_lex in t:

            # expand parameter maps for new lexical entries and update parse structures
            # so that roots have correct surface form idxs (which may not have been assigned
            # yet at the time of parsing)
            for form, lex in [[y, y_lex], [z, z_lex]]:
                for surface_form, sem_node in lex:

                    # add this surface form and sem node to the lexicon and tie them together
                    if type(surface_form) is str and surface_form not in self.lexicon.surface_forms:
                        # print "Parameters: adding new surface form '"+surface_form+"'"  # DEBUG
                        self.lexicon.surface_forms.append(surface_form)
                    sf_idx = self.lexicon.surface_forms.index(surface_form)
                    if sem_node not in self.lexicon.semantic_forms:
                        # print "Parameters: adding new semantic form '"+str(sem_node)+"'"  # DEBUG
                        self.lexicon.semantic_forms.append(sem_node)
                    sem_idx = self.lexicon.semantic_forms.index(sem_node)
                    if sf_idx == len(self.lexicon.entries):
                        self.lexicon.entries.append([])
                    if sem_idx not in self.lexicon.entries[sf_idx]:
                        # print "Parameters: adding new entry "+str(sf_idx)+"=>"+str(sem_idx)  # DEBUG
                        self.lexicon.entries[sf_idx].append(sem_idx)

                    # update count data structures given new entry
                    key = (self.lexicon.semantic_forms[sem_idx].category, sf_idx)
                    if key not in self._CCG_given_token_counts:
                        self._CCG_given_token_counts[key] = 1.0
                    # note that no new CCG production rules can be generated from current procedure
                    key = (sf_idx, sem_idx)
                    if key not in self._lexicon_entry_given_token_counts:
                        self._lexicon_entry_given_token_counts[key] = 1.0

                # update form leaves with new surface idx as needed
                form_leaves = form.get_leaves()
                for leaf in form_leaves:
                    if type(leaf.surface_form) is str:
                        leaf.surface_form = self.lexicon.surface_forms.index(leaf.surface_form)

            # do perceptron-style updates of counts
            parameter_extractors = [count_ccg_surface_form_pairs,
                                    count_ccg_productions,
                                    count_lexical_entries,
                                    self.count_semantics,
                                    count_ccg_root]
            parameter_structures = [self._CCG_given_token_counts,
                                    self._CCG_production_counts,
                                    self._lexicon_entry_given_token_counts,
                                    self._semantic_counts,
                                    self._CCG_root_counts]
            if self.use_language_model:
                parameter_extractors.append(self.count_token_bigrams)
                parameter_structures.append(self._token_given_token_counts)
            for i in range(0, len(parameter_structures)):
                y_keys = parameter_extractors[i](y)
                z_keys = parameter_extractors[i](z)
                seen_keys = []
                for z_key in z_keys:
                    z_val = z_keys[z_key]
                    if z_key in y_keys:
                        y_val = y_keys[z_key]
                        seen_keys.append(z_key)
                    else:
                        y_val = 0
                    if z_key not in parameter_structures[i]:
                        parameter_structures[i][z_key] = 0
                    parameter_structures[i][z_key] += (z_val - y_val) / (z_val + y_val)  # NEW
                for y_key in y_keys:
                    if y_key in seen_keys:
                        continue
                    y_val = y_keys[y_key]
                    z_val = 0
                    if y_key not in parameter_structures[i]:
                        parameter_structures[i][y_key] = 0
                    parameter_structures[i][y_key] += (z_val - y_val) / (z_val + y_val)  # NEW

        # print "_token_given_token_counts: "+str(self._token_given_token_counts)  # DEBUG
        # print "_CCG_given_token_counts: "+str(self._CCG_given_token_counts)  # DEBUG
        # print "_CCG_production_counts: "+str(self._CCG_production_counts)  # DEBUG
        # print "_lexicon_entry_counts: "+str(self._lexicon_entry_given_token_counts)  # DEBUG
        # print "_semantic_counts: "+str([str((self.ontology.preds[pred] if type(pred) is int else pred,
        #                                      self.ontology.preds[arg] if type(arg) is int else arg,
        #                                      str(pos)))+": " +
        #                                str(self._semantic_counts[(pred, arg, pos)])
        #                                for pred, arg, pos in self._semantic_counts])  # DEBUG
        # print {self.lexicon.compose_str_from_category(idx): self._CCG_root_counts[idx]
        #        for idx in self._CCG_root_counts}  # DEBUG

        # update probabilities given new counts
        self.update_probabilities()

        # print "token_given_token: "+str(self.token_given_token)  # DEBUG
        # print "CCG_given_token: "+str(self.CCG_given_token)  # DEBUG
        # print "CCG_production: "+str(self.CCG_production)  # DEBUG
        # print "lexicon_entry: "+str(self.lexicon_entry_given_token)  # DEBUG
        # print "semantic: "+str([str((self.ontology.preds[pred] if type(pred) is int else pred,
        #                             self.ontology.preds[arg] if type(arg) is int else arg,
        #                             str(pos)))+": " +
        #                         str(self.semantic[(pred, arg, pos)])
        #                         for pred, arg, pos in self.semantic])  # DEBUG
        # print {self.lexicon.compose_str_from_category(idx): self.CCG_root[idx]
        #        for idx in self.CCG_root if idx > -1}  # DEBUG
        # print "unseen prob: " + str(self.CCG_root[-1])  # DEBUG


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


# take in ParseNode y and return the CCG of its root
def count_ccg_root(y):
    return {y.node.category: 1}


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
    def __init__(self, ont, lex, use_language_model=False, lexicon_weight=1.0):

        # resources given on instantiation
        self.ontology = ont
        self.lexicon = lex
        self.use_language_model = use_language_model

        # type-raise bare nouns in lexicon
        self.type_raised = {}  # map from semantic form idx to their type-raised form idx
        self.type_raise_bare_nouns()

        # model parameter values
        self.theta = Parameters(ont, lex, use_language_model=use_language_model, lexicon_weight=lexicon_weight)

        # additional linguistic information and parameters
        # TODO: read this from configuration files or have user specify it on instantiation
        self.commutative_idxs = [self.ontology.preds.index('and'), self.ontology.preds.index('or')]
        self.max_multiword_expression = 2  # max span of a multi-word expression to be considered during tokenization
        self.max_new_senses_per_utterance = 2  # max number of new word senses that can be induced on a training example
        self.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
        self.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried
        self.max_expansions_per_non_terminal = 10  # decides how many expansions to store per CKY cell

        # behavioral parameters
        self.safety = True  # set to False once confident about node combination functions' correctness

        # cache
        self.cached_combinations = {}  # indexed by left, then right node, value at result

    # access language model parameters to get a language score for a given parse node y
    # parse node leaves with string semantic forms are assumed to be unknown tokens
    def get_language_model_score(self, y):
        if y is None :
            return -sys.maxint
        t = [l.surface_form for l in y.get_leaves()]
        for t_idx in range(0, len(t)):
            if type(t[t_idx]) is str:
                t[t_idx] = -1
        score = 0
        t.insert(0, -2)
        t.append(-3)
        for t_idx in range(0, len(t)-1):
            key = (t[t_idx], t[t_idx+1])
            score += self.theta.token_given_token[key] if key in self.theta.token_given_token else 0.0
        return score

    # perform type-raising on leaf-level lexicon entries
    # this alters the given lexicon
    def type_raise_bare_nouns(self):
        bare_noun_cat_idx = self.lexicon.categories.index('N')
        raised_cat_idx = self.lexicon.categories.index([bare_noun_cat_idx, 1, bare_noun_cat_idx])
        e_idx = self.ontology.types.index('e')
        e_to_t_idx = self.ontology.types.index([self.ontology.types.index('e'), self.ontology.types.index('t')])
        to_add = []
        for sf_idx in range(0, len(self.lexicon.surface_forms)):
            for sem_idx in self.lexicon.entries[sf_idx]:
                sem = self.lexicon.semantic_forms[sem_idx]
                if (sem.category == bare_noun_cat_idx and not sem.is_lambda and
                        self.ontology.entries[sem.idx] == e_to_t_idx and sem.children is None):
                    # ensure there isn't already a raised predicate matching this bare noun
                    already_raised = False
                    for alt_idx in self.lexicon.entries[sf_idx]:
                        alt = self.lexicon.semantic_forms[alt_idx]
                        if (alt.category == raised_cat_idx and alt.is_lambda and
                                alt.type == e_idx and
                                not alt.children[0].is_lambda and
                                self.ontology.entries[alt.children[0].idx] == e_to_t_idx and
                                alt.children[0].children[0].is_lambda_instantiation):
                            already_raised = True
                            break
                    if not already_raised:
                        raised_sem = SemanticNode.SemanticNode(None, e_idx, raised_cat_idx,
                                                               True, lambda_name=0, is_lambda_instantiation=True)
                        raised_pred = copy.deepcopy(sem)
                        raised_pred.parent = raised_sem
                        lambda_inst = SemanticNode.SemanticNode(raised_pred, e_idx, bare_noun_cat_idx,
                                                                True, lambda_name=0, is_lambda_instantiation=False)
                        raised_pred.children = [lambda_inst]
                        raised_sem.children = [raised_pred]
                        to_add.append([sf_idx, sem_idx, raised_sem])
        for sf_idx, sem_idx, sem in to_add:
            # print "type_raise_bare_nouns raising: '"+self.lexicon.surface_forms[sf_idx] + \
            #       "':- "+self.print_parse(sem, show_category=True)  # DEBUG
            self.lexicon.semantic_forms.append(sem)
            self.lexicon.entries[sf_idx].append(len(self.lexicon.semantic_forms)-1)
            self.type_raised[sem_idx] = len(self.lexicon.semantic_forms)-1

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
    def train_learner_on_semantic_forms(self, d, epochs=10, reranker_beam=1):
        for e in range(0, epochs):
            print "epoch " + str(e)  # DEBUG
            t, failures = self.get_training_pairs(d, reranker_beam=reranker_beam)
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
    # training pairs in t are of form (x, y_chosen, y_correct, chosen_lex_entries, correct_lex_entries)
    # k determines how many parses to get for re-ranking
    # beam determines how many cky_trees to look through before giving up on a given input
    def get_training_pairs(self, d, reranker_beam=1):
        t = []
        num_trainable = 0
        num_matches = 0
        num_fails = 0
        num_genlex_only = 0
        for [x, y] in d:
            correct_parse = None
            correct_new_lexicon_entries = []
            cky_parse_generator = self.most_likely_cky_parse(x, reranker_beam=reranker_beam, known_root=y)
            chosen_parse, chosen_score, chosen_new_lexicon_entries = next(cky_parse_generator)
            current_parse = chosen_parse
            correct_score = chosen_score
            current_new_lexicon_entries = chosen_new_lexicon_entries
            match = False
            first = True
            if chosen_parse is None:
                # print "WARNING: could not find valid parse for '" + x + "' during training"
                num_fails += 1
                continue
            while correct_parse is None and current_parse is not None:
                if y.equal_allowing_commutativity(
                        current_parse.node, self.commutative_idxs, ontology=self.ontology):
                    correct_parse = current_parse
                    correct_new_lexicon_entries = current_new_lexicon_entries
                    if first:
                        match = True
                        num_matches += 1
                    else:
                        num_trainable += 1
                    break
                first = False
                current_parse, correct_score, current_new_lexicon_entries = next(cky_parse_generator)
            if correct_parse is None:
                # print "WARNING: could not find correct parse for '"+str(x)+"' during training"
                num_fails += 1
                continue
            # print "\tx: "+str(x)  # DEBUG
            # print "\t\tchosen_parse: "+self.print_parse(chosen_parse.node, show_category=True)  # DEBUG
            # print "\t\tchosen_score: "+str(chosen_score)  # DEBUG
            # print "\t\tchosen_new_lexicon_entries: "  # DEBUG
            # for sf, sem in chosen_new_lexicon_entries:  # DEBUG
            #     print "\t\t\t'"+sf+"':- "+self.print_parse(sem, show_category=True)  # DEBUG
            if not match or len(correct_new_lexicon_entries) > 0:
                if len(correct_new_lexicon_entries) > 0:
                    num_genlex_only += 1
                # print "\ttraining example generated:"  # DEBUG
                # print "\t\tcorrect_parse: "+self.print_parse(correct_parse.node, show_category=True)  # DEBUG
                # print "\t\tcorrect_score: "+str(correct_score)  # DEBUG
                # print "\t\tcorrect_new_lexicon_entries: "  # DEBUG
                # for sf, sem in correct_new_lexicon_entries:  # DEBUG
                #     print "\t\t\t'"+sf+"':- "+self.print_parse(sem, show_category=True)  # DEBUG
                # print "\t\ty: "+self.print_parse(y, show_category=True)  # DEBUG
                t.append([x, chosen_parse, correct_parse, chosen_new_lexicon_entries, correct_new_lexicon_entries])
        print "\tmatched "+str(num_matches)+"/"+str(len(d))  # DEBUG
        print "\ttrained "+str(num_trainable)+"/"+str(len(d))  # DEBUG
        print "\tgenlex only "+str(num_genlex_only)+"/"+str(len(d))  # DEBUG
        print "\tfailed "+str(num_fails)+"/"+str(len(d))  # DEBUG
        return t, num_fails

    # yields the next most likely CKY parse of input string s
    # if the root of the tree is known (during supervised training, for example),
    # providing it as an argument to this method allows top-down generation
    # to find new lexical entries for surface forms not yet recognized
    def most_likely_cky_parse(self, s, reranker_beam=1, known_root=None):
        if len(s) == 0:
            raise AssertionError("Cannot parse provided string of length zero")

        tk_seqs = self.tokenize(s)
        # print "number of token sequences for '"+s+"': "+str(len(tk_seqs))  # DEBUG
        # print "tk_seqs: "+str(tk_seqs)  # DEBUG

        # calculate skip scores of each token in each sequence
        skip_scores = []
        for tks in tk_seqs:
            skip_score = {}
            if len(tks) > 1:
                for idx in range(0, len(tks)):
                    if tks[idx] in self.lexicon.surface_forms:
                        # TODO: should parameterize prior on surface forms
                        skip_score[idx] = math.log((len(tks)-1)/float(len(tks)))
                    else:
                        skip_score[idx] = 0  # no penalty when skipping totally unknown tokens
                        sub_tokens = tks[idx].split(' ')
                        for st in sub_tokens:
                            if st in self.lexicon.surface_forms:
                                skip_score[idx] = math.log((len(tks)-1)/float(len(tks)))
            skip_scores.append(skip_score)
        # print "skip_scores: "+str(skip_scores)  # DEBUG

        skips_allowed = 0
        while skips_allowed < 2:  # skip at most one token in a given sequence at a time

            # calculate token sequence variations with number of skips allowed
            tk_seqs_with_skips = []
            for seq_idx in range(0, len(tk_seqs)):
                # add an entry for each combination of skipped tokens ordered by score
                ordered_skip_sequences = None
                if skips_allowed == 0:
                    ordered_skip_sequences = [tk_seqs[seq_idx][:]]
                elif skips_allowed >= len(tk_seqs[seq_idx]):
                    ordered_skip_sequences = []  # no valid subset of this sequence; all tokens would be skipped
                elif skips_allowed == 1:
                    skip_sequences = []
                    for idx, score in sorted(skip_scores[seq_idx].items(), key=operator.itemgetter(1), reverse=True):
                        skip_sequences.append([tk_seqs[seq_idx][t]
                                              for t in range(0, len(tk_seqs[seq_idx]))
                                              if t != idx])
                    ordered_skip_sequences = skip_sequences
                tk_seqs_with_skips.append(ordered_skip_sequences)

            curr_tk_seqs = None
            while curr_tk_seqs is None or len(curr_tk_seqs) > 0:

                # pop first most likely skip from each possibility
                curr_tk_seqs = []
                for idx in range(0, len(tk_seqs)):
                    if len(tk_seqs_with_skips[idx]) > 0:
                        curr_tk_seqs.append(tk_seqs_with_skips[idx].pop(0))
                if len(curr_tk_seqs) == 0:
                    continue
                # print "curr_tk_seqs: "+str(curr_tk_seqs)  # DEBUG

                # create generator for current sequence set and get most likely parses

                ccg_parse_tree_generator = self.most_likely_ccg_parse_tree(curr_tk_seqs)
                # get next most likely CCG parse tree out of CKY algorithm
                ccg_tree, tree_score, tks = next(ccg_parse_tree_generator)
                # ccg_tree indexed by spans (i, j) valued at [CCG category, left span, right span]
                while ccg_tree is not None:

                    # print "ccg tree: "+str(tree_score)  # DEBUG
                    # for span in ccg_tree:  # DEBUG
                        # print str(span) + ": [" + self.lexicon.compose_str_from_category(ccg_tree[span][0]) + \
                            # "," + str(ccg_tree[span][1]) + "," + str(ccg_tree[span][2]) + "]"  # DEBUG

                    # get next most likely assignment of semantics to given CCG categories
                    semantic_assignment_generator = self.most_likely_semantic_leaves(tks, ccg_tree,
                                                                                     known_root=known_root)

                    # use discriminative re-ranking to pull next most likely cky parse given leaf generator
                    parse_tree_generator = self.most_likely_reranked_cky_parse(ccg_tree, semantic_assignment_generator,
                                                                               reranker_beam, known_root=known_root)
                    parse_tree, parse_score, new_lexicon_entries = next(parse_tree_generator)
                    while parse_tree is not None:
                        yield parse_tree, parse_score + tree_score, new_lexicon_entries
                        parse_tree, parse_score, new_lexicon_entries = next(parse_tree_generator)

                    ccg_tree, tree_score, tks = next(ccg_parse_tree_generator)

            skips_allowed += 1

            # if we know the root, we don't need to use skipping at all since we'll use generation techniques
            # to fill in missing entries for training; so we break here
            if known_root is not None:
                break

        # out of parse trees to try
        yield None, neg_inf, []

    # yields the next most likely parse tree after re-ranking in beam k
    # searches over leaf assignments in beam given a leaf assignment generator
    def most_likely_reranked_cky_parse(self, ccg_tree, semantic_assignment_generator, k, known_root=None):

        # get up to top k candidates
        candidates = []  # list of trees
        scores = []  # list of scores after discriminative re-ranking
        new_lex = []  # new lexical entries associated with each
        for curr_leaves, curr_leaves_score in semantic_assignment_generator:
            curr_generator = self.most_likely_tree_generator(curr_leaves, ccg_tree, sem_root=known_root)
            # print "curr_leaves: "+str([self.print_parse(cl.node, show_category=True) for cl in curr_leaves])  # DEBUG
            for curr_tree, curr_new_lex in curr_generator:
                # print "...added candidate"  # DEBUG
                candidates.append(curr_tree)
                scores.append(self.theta.get_semantic_score(curr_tree) + curr_leaves_score)
                new_lex.append(curr_new_lex)
            if len(candidates) > k:
                break

        # for idx in range(0, len(candidates)):  # DEBUG
        #     print "candidate: "+self.print_parse(candidates[idx].node, show_category=True)  # DEBUG
        #     print "\tscore: "+str(scores[idx])  # DEBUG

        # yield highest scoring tree and replace it via generation until no leaves remain
        for curr_leaves, curr_leaves_score in semantic_assignment_generator:
            curr_generator = self.most_likely_tree_generator(curr_leaves, ccg_tree, sem_root=known_root)
            for curr_tree, curr_new_lex in curr_generator:
                best_idx = 0
                for idx in range(0, len(candidates)):
                    if scores[best_idx] < scores[idx]:
                        best_idx = idx
                yield candidates[idx], scores[idx], new_lex[idx]
                del candidates[idx]
                del scores[idx]
                del new_lex[idx]
                candidates.append(curr_tree)
                scores.append(self.theta.get_semantic_score(curr_tree) + curr_leaves_score)
                new_lex.append(curr_new_lex)

        # yield remaining best candidates in order
        score_dict = {idx: scores[idx] for idx in range(0, len(scores))}
        for idx, score in sorted(score_dict.items(), key=operator.itemgetter(1), reverse=True):
            yield candidates[idx], score, new_lex[idx]

        # out of candidates
        yield None, neg_inf, []

    # yields the next most likely assignment of semantic values to ccg nodes
    # returns None if leaf assignments cannot propagate to root
    # returns None if no assignment to missing leaf entries will allow propagation to root
    def most_likely_tree_generator(self, parse_leaves, ccg_tree, sem_root=None):

        # print "most_likely_tree_generator: called for ccg_tree: "+str(ccg_tree)  # DEBUG
        parse_roots, parse_leaves_keys = self.form_root_from_leaves(parse_leaves, ccg_tree)
        # print "parse_roots: "+str([self.print_parse(p.node) for p in parse_roots])  # DEBUG

        # yield parse and total score (leaves+syntax) if structure matches
        # set of new lexical entries required is empty in this case
        if len(parse_roots) == 1 and parse_roots[0].node is not None:
            yield parse_roots[0], []  # the ParseNode root of the finished parse tree

        # if structure does not match and there are None nodes, perform top-down generation from
        # supervised root (if available) to fill in unknown semantic gaps given CKY structure of
        # the ccg_tree
        parse_leaves_nodes = [pl.node for pl in parse_roots]
        if sem_root is not None and None in parse_leaves_nodes:

            # print "trying top-down parsing..."  # DEBUG
            top_down_chart = {}
            root_key = (0, 1)
            for entry in ccg_tree:
                if entry[0] == 0 and entry[1] > root_key[1]:
                    root_key = entry
            top_down_chart[root_key] = ParseNode.ParseNode(None, sem_root)

            topdown_tree_generator = self.get_most_likely_tree_from_root(top_down_chart[root_key],
                                                                         root_key,
                                                                         ccg_tree,
                                                                         parse_leaves_keys)
            for topdown_root, topdown_score in topdown_tree_generator:

                new_lex_entries = []  # values (surface form str, sem node)
                topdown_leaves = topdown_root.get_leaves()
                if len(topdown_leaves) == len(parse_roots):  # possible match was found in reverse parsing
                    match = True
                    candidate_parse_leaves = parse_roots[:]
                    for idx in range(0, len(candidate_parse_leaves)):
                        if candidate_parse_leaves[idx].node is None:
                            sf = candidate_parse_leaves[idx].surface_form if \
                                type(candidate_parse_leaves[idx].surface_form) is str else \
                                self.lexicon.surface_forms[candidate_parse_leaves[idx].surface_form]
                            new_lex_entries.append([sf, topdown_leaves[idx].node])
                            candidate_parse_leaves[idx] = topdown_leaves[idx]
                            candidate_parse_leaves[idx].surface_form = sf
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
                        # print "new_lex_entries: "  # DEBUG
                        # for nle in new_lex_entries:  # DEBUG
                        #     print nle[0]+" :- "+self.print_parse(nle[1], show_category=True)  # DEBUG

                        candidate_parse_leaves, candidate_leaf_spans = \
                            self.form_root_from_leaves(candidate_parse_leaves, ccg_tree)

                        # print "candidate_parse_leaves: "+str(candidate_parse_leaves)  # DEBUG
                        # for cpl in candidate_parse_leaves:  # DEBUG
                        #     print self.print_parse(cpl.node, show_category=True)  # DEBUG

                        # the ParseNode root of the finished parse tree
                        if len(candidate_parse_leaves) == 1:
                            yield candidate_parse_leaves[0], new_lex_entries

    # given parse leaves, form as close to root as possible
    def form_root_from_leaves(self, parse_leaves, ccg_tree):

        # try to build parse tree from lexical assignments guided by CKY structure
        spans = []
        for key in ccg_tree:
            if ccg_tree[key][1] is None and ccg_tree[key][2] is None:
                idx = 0
                for idx in range(0, len(spans)+1):  # insertion sort to get spans in linear order
                    if idx == len(spans) or key[0] < spans[idx][0]:
                        break
                spans.insert(idx, key)
        # print "form_root_from_leaves spans: "+str(spans)  # DEBUG
        found_combination = True
        while len(parse_leaves) > 1 and found_combination:

            # print "parse leaves: "  str(parse_leaves)  # DEBUG
            # for i in range(0, len(parse_leaves)):  # DEBUG
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

        # print "get_most_likely_tree_from_root called"  # DEBUG
        # print "root_key: "+str(root_key)  # DEBUG
        # print "ccg_tree: "+str(ccg_tree)  # DEBUG

        # if root key is the only entry in the chart, can only associate it with the known parse root
        if len(ccg_tree.keys()) == 1:
            yield parse_root, 0
        else:

            # greedily take children with best score until we get a category match
            children_generator = self.get_most_likely_children_from_root(parse_root.node)
            for children, children_score in children_generator:
                # when we find children with syntax matching ccg tree, save and see whether either should expand
                if children[0].category == ccg_tree[ccg_tree[root_key][1]][0] and \
                   children[1].category == ccg_tree[ccg_tree[root_key][2]][0]:
                    # print "...category match"  # DEBUG
                    parse_root.children = []
                    for c in range(0, 2):  # save calculated semantic children wrapped in ParseNodes
                        parse_root.children.append(ParseNode.ParseNode(parse_root, children[c]))
                    if ccg_tree[root_key][1] in known_leaf_keys and ccg_tree[root_key][2] in known_leaf_keys:
                        # print "...get_most_likely_tree_from_root yielding two known leaf keys"  # DEBUG
                        yield parse_root, children_score
                    for c in range(0, 2):  # expand children
                        subtree_generators = [self.get_most_likely_tree_from_root(parse_root.children[c],
                                                                                  ccg_tree[root_key][1+c],
                                                                                  ccg_tree,
                                                                                  known_leaf_keys)
                                              for c in range(0, 2)]
                        if ccg_tree[root_key][1+c] not in known_leaf_keys:
                            for child1, score1 in subtree_generators[c]:
                                parse_root.children[c] = child1
                                if ccg_tree[root_key][1+((c+1) % 2)] not in known_leaf_keys:
                                    for child2, score2 in subtree_generators[(c+1) % 2]:
                                        parse_root.children[(c+1) % 2] = child2
                                        # print "...get_most_likely_tree_from_root yielding deeper children"  # DEBUG
                                        yield parse_root, children_score+score1+score2
                                else:
                                    # print "...get_most_likely_tree_from_root yielding deeper child"  # DEBUG
                                    yield parse_root, children_score+score1

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

    # yields next most likely assignment of semantic values to ccg tree leaves
    def most_likely_semantic_leaves(self, tks, ccg_tree, known_root=None):

        # get syntax categories for tree leaves
        leaf_categories = []
        curr_idx = 0
        spans = []
        while curr_idx < len(tks):
            for span in range(0, self.max_multiword_expression):
                key = (curr_idx, curr_idx+span+1)
                if key in ccg_tree:
                    leaf_categories.append(ccg_tree[key][0])
                    spans.append(key)
                    curr_idx += span+1
                    break
        # print "most_likely_semantic_leaves: called for tks "+str(tks)  # DEBUG

        # get possible semantic forms for each syntax/surface combination represented by leaf_categories and tks
        semantic_candidates = []
        expressions = []
        for idx in range(0, len(spans)):
            exp = ' '.join([tks[t] for t in range(spans[idx][0], spans[idx][1])])
            expressions.append(exp)
            if exp in self.lexicon.surface_forms:
                semantic_candidates.append([sem_idx for sem_idx
                                            in self.lexicon.entries[self.lexicon.surface_forms.index(exp)]
                                            if self.lexicon.semantic_forms[sem_idx].category == leaf_categories[idx]])
            else:  # unknown surface form
                # if the root is known, semantic candidates will be generated after the fact top-down
                if known_root is not None:
                    semantic_candidates.append([])
                # if the root isn't known, the best we can do is try all semantic forms that match the given
                # syntax; ie look for synonyms in the lexicon; search space is less restricted
                else:
                    semantic_candidates.append([sem_idx for sem_idx
                                                in range(0, len(self.lexicon.semantic_forms))
                                                if self.lexicon.semantic_forms[sem_idx].category
                                                == leaf_categories[idx]])
                    semantic_candidates[-1].append(None)

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
            for idx in range(0, len(spans)):
                if len(semantic_candidates[idx]) == 0:
                    assignments.append(None)  # no known semantic assignment
                else:
                    assignment_idx = curr % len(semantic_candidates[idx])
                    assignments.append(assignment_idx)
                    if expressions[idx] in self.lexicon.surface_forms:  # surface form is in lexicon
                        key = (semantic_candidates[idx][assignment_idx],
                               self.lexicon.surface_forms.index(expressions[idx]))
                        score += self.theta.lexicon_entry_given_token[key] \
                            if key in self.theta.lexicon_entry_given_token else \
                            self.theta.lexicon_entry_given_token[(semantic_candidates[idx][assignment_idx], -1)]
                    else:  # surface form not in lexicon; we're just grasping at synonyms
                        # we can use an arbitrary semantic form idx since this is a uniform probability given
                        # unknown surface form
                        score += self.theta.lexicon_entry_given_token[(0, -1)]
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
                                         if assignment[idx] is not None and
                                         semantic_candidates[idx][assignment[idx]] is not None else None,
                                         surface_form=self.lexicon.surface_forms.index(expressions[idx])
                                         if expressions[idx] in self.lexicon.surface_forms else expressions[idx],
                                         semantic_form=semantic_candidates[idx][assignment[idx]]
                                         if assignment[idx] is not None and
                                         semantic_candidates[idx][assignment[idx]] is not None else None)
                     for idx in range(0, len(spans))]
            yield nodes, score

    # yields the next most likely ccg parse tree given a set of possible token sequences
    # finds the best parse tree given each token sequence and returns in-order the one
    # with the highest score
    def most_likely_ccg_parse_tree(self, tk_seqs):
        ccg_parse_tree_generators = [self.most_likely_ccg_parse_tree_given_tokens(tks)
                                     for tks in tk_seqs]
        best_per_seq = [next(ccg_parse_tree_generators[idx])
                        for idx in range(0, len(tk_seqs))]
        leaf_sense_limits = [0 for _ in range(0, len(tk_seqs))]

        # remove candidates for which we have no surface forms at all
        while [None, neg_inf] in best_per_seq:
            idx = best_per_seq.index([None, neg_inf])
            del best_per_seq[idx]
            del ccg_parse_tree_generators[idx]
            del leaf_sense_limits[idx]
            del tk_seqs[idx]

        best_idx = 0
        best_score = neg_inf
        tied_idxs = [0]
        while len(best_per_seq) > 0:
            for idx in range(0, len(tk_seqs)):  # select the highest-scoring ccg parse(s)
                if best_per_seq[idx][1] > best_score:
                    best_idx = idx
                    tied_idxs = [idx]
                    best_score = best_per_seq[idx][1]
                elif best_per_seq[idx][1] == best_score:
                    tied_idxs.append(idx)
            for idx in tied_idxs:  # among those tied in score, select ccg parse with most tokens (fewest multi-word)
                if len(tk_seqs[idx]) > len(tk_seqs[best_idx]):
                    best_idx = idx
            # print "most_likely_ccg_parse_tree: yielding " + \
                # str(best_per_seq[best_idx][0])+" with score "+str(best_score) + \
                # " from tokens "+str(tk_seqs[best_idx])  # DEBUG
            yield best_per_seq[best_idx][0], best_score, tk_seqs[best_idx]
            candidate, score = next(ccg_parse_tree_generators[best_idx])
            empty = True
            if candidate is not None:
                best_per_seq[best_idx] = [candidate, score]
                empty = False
            else:
                if leaf_sense_limits[best_idx] < self.max_new_senses_per_utterance:
                    leaf_sense_limits[best_idx] += 1
                    ccg_parse_tree_generators[best_idx] = self.most_likely_ccg_parse_tree_given_tokens(
                        tk_seqs[best_idx], new_sense_leaf_limit=leaf_sense_limits[best_idx])
                    candidate, score = next(ccg_parse_tree_generators[best_idx])
                    if candidate is not None:
                        best_per_seq[best_idx] = [candidate, score]
                        empty = False
            if empty:
                del best_per_seq[best_idx]
                del ccg_parse_tree_generators[best_idx]
                del leaf_sense_limits[best_idx]
                del tk_seqs[best_idx]
                best_idx = 0
                tied_idxs = [0]
        yield None, neg_inf, None

    # yields the next most likely ccg parse tree given a set of tokens
    def most_likely_ccg_parse_tree_given_tokens(self, tks, new_sense_leaf_limit=0):

        # print "most_likely_ccg_parse_tree_given_tokens initialized with tks="+str(tks) + \
            # ", new_sense_leaf_limit="+str(new_sense_leaf_limit)  # DEBUG
        # debug_chr = raw_input()

        # indexed by position in span (i, j) in parse tree
        # value is list of tuples [CCG category, [left key, left index], [right key, right index], score]
        chart = {}

        # values are positions in span (i, j) in parse tree
        # value is present if this span is missing from the lexicon and thus, in the chart, eligible to
        # receive arbitrary matching syntax categories given neighbors during CKY
        missing = []

        # values are positions in span (i, j) in parse tree
        # value is present if span is present in lexicon but eligible to receive arbitrary matching syntax
        # categories given neighbors during CKY for purposes of multi-sense detection
        sense_leaf_keys = []

        # assign leaf values
        max_entries = [None, None]  # track which span has the most lexical entries
        for span in range(0, self.max_multiword_expression):
            for idx in range(0, len(tks)-span):
                pos = (idx, idx+span+1)
                chart[pos] = []
                exp = ' '.join([tks[t] for t in range(pos[0], pos[1])])
                if exp in self.lexicon.surface_forms:
                    sf_idx = self.lexicon.surface_forms.index(exp)
                    cats = [self.lexicon.semantic_forms[sem_idx].category for sem_idx in self.lexicon.entries[sf_idx]]
                    for cat_idx in cats:
                        score = self.theta.CCG_given_token[(cat_idx, sf_idx)] \
                            if (cat_idx, sf_idx) in self.theta.CCG_given_token \
                            else self.theta.CCG_given_token[(cat_idx, -1)]
                        score *= span  # NEW
                        chart[pos].append([cat_idx, None, None, score])
                    if max_entries[1] is None or max_entries[1] < len(self.lexicon.entries[sf_idx]):
                        max_entries[0] = pos
                        max_entries[1] = len(self.lexicon.entries[sf_idx])
                else:
                    missing.append(pos)

        # assign sense leaves based on max entries
        for i in range(0, new_sense_leaf_limit):
            if max_entries[0] is not None:
                sense_leaf_keys.append(max_entries[0])
            max_entries = [None, None]
            for span in range(1, self.max_multiword_expression):
                for idx in range(0, len(tks)-span):
                    pos = (idx, idx+span+1)
                    exp = ' '.join([tks[t] for t in range(pos[0], pos[1])])
                    if pos not in missing:
                        sf_idx = self.lexicon.surface_forms.index(exp)
                        if (pos not in sense_leaf_keys and
                                (max_entries[0] is not None or max_entries[1] < len(self.lexicon.entries[sf_idx]))):
                            max_entries[0] = pos
                            max_entries[1] = len(self.lexicon.entries[sf_idx])

        # print "leaf chart: "+str(chart)  # DEBUG
        # print "leaf missing: "+str(missing)  # DEBUG
        # _ = raw_input()  # DEBUG

        # populate chart for length 1 utterance
        if len(tks) == 1:
            # chart entries should be all top-level CCG categories (those that don't take arguments)
            pos = (0, 1)
            for cat_idx in range(0, len(self.lexicon.categories)):
                if type(self.lexicon.categories[cat_idx]) is str:
                    score = self.theta.CCG_given_token[(cat_idx, -1)]
                    chart[pos].append([cat_idx, None, None, score])

        # populate chart using CKY
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
                        # if debug_chr == 'y' : # DEBUG
                        #     print "l: "+str(l)+", cat="+self.lexicon.compose_str_from_category(l[0])  # DEBUG
                        for r_idx in range(0, len(right)):
                            r = right[r_idx]
                            # if debug_chr == 'y' : # DEBUG
                            #     print "r: "+str(r)+", cat="+self.lexicon.compose_str_from_category(r[0])  # DEBUG
                            for prod in self.theta.CCG_production:
                                # if debug_chr == 'y' : # DEBUG
                                #     print "prod: "+str([self.lexicon.compose_str_from_category(c)
                                #  for c in prod])  # DEBUG
                                if prod[1] == l[0] and prod[2] == r[0]:
                                    new_score = self.theta.CCG_production[prod] + l[3] + r[3]
                                    # Find all expansions of this non-terminal in the cell
                                    expansions = [expansion for expansion in chart[key] if expansion[0] == prod[0]]
                                    if len(expansions) < self.max_expansions_per_non_terminal - 1:
                                        chart[key].append([prod[0], [l_key, l_idx], [r_key, r_idx], new_score])
                                    else:
                                        # Sort expansions in order of score
                                        scores_and_expansions = [(expansion[-1], expansion) for expansion in expansions]
                                        scores_and_expansions.sort() 
                                        if new_score > scores_and_expansions[0][0]:
                                            # Remove the least scoring existing expansion and
                                            # add the one just found
                                            chart[key].remove(scores_and_expansions[0][1])
                                            chart[key].append([prod[0], [l_key, l_idx], [r_key, r_idx], new_score])
                                            
                                    # print "new chart entry "+str(key)+" : "+str(chart[key][-1])  # DEBUG
                    lr_keys = [l_key, r_key]
                    lr_dirs = [left, right]
                    lr_idxs = [1, 2]
                    for branch_idx in range(0, 2):
                        m_idx = branch_idx
                        p_idx = 0 if branch_idx == 1 else 1
                        if ((lr_keys[m_idx] in sense_leaf_keys or lr_keys[m_idx] in missing)
                                and lr_keys[p_idx] not in missing):
                            # print "searching for matches for missing "+str(lr_keys[m_idx]) + \
                            #       " against present "+str(lr_keys[p_idx])  # DEBUG
                            # investigate syntax for leaf m that can consume or be consumed by p
                            # because of rewriting the CCG rules in CNF form, if a production rule is present
                            # it means that a consumation or merge can happen and we can be agnostic to how
                            for idx in range(0, len(lr_dirs[p_idx])):
                                p = lr_dirs[p_idx][idx]
                                hypothesis_categories_added = 0
                                ccg_productions = self.theta.CCG_production.keys()[:]
                                random.shuffle(ccg_productions)
                                for prod in ccg_productions:
                                    # leaf m can be be combine with p
                                    if prod[lr_idxs[p_idx]] == p[0]:
                                        # give probability of unknown token assigned to this CCG category
                                        m_score = self.theta.CCG_given_token[(prod[lr_idxs[m_idx]], -1)]
                                        m = [prod[lr_idxs[m_idx]], None, None, m_score]
                                        chart[lr_keys[m_idx]].append(m)
                                        new_score = self.theta.CCG_production[prod] + p[3] + m[3]
                                        new_entry = [prod[0], None, None, new_score]
                                        new_entry[lr_idxs[m_idx]] = [lr_keys[m_idx], len(chart[lr_keys[m_idx]])-1]
                                        new_entry[lr_idxs[p_idx]] = [lr_keys[p_idx], idx]
                                        chart[key].append(new_entry)
                                        # print "new chart entry "+str(key)+" : "+str(chart[key][-1])  # DEBUG
                                        hypothesis_categories_added += 1
                                        if hypothesis_categories_added == \
                                                self.max_hypothesis_categories_for_unknown_token_beam:
                                            break

                    # print "chart: "+str(chart)  # DEBUG
                    # _ = raw_input()  # DEBUG

        # print "finished chart: "+str(chart)  # DEBUG
        # _ = raw_input()

        # weight trees using prior on root CCG node
        key = (0, len(tks))
        for i in range(0, len(chart[key])):
            chart[key][i][3] += (self.theta.CCG_root[chart[key][i][0]]
                                 if chart[key][i][0] in self.theta.CCG_root
                                 else self.theta.CCG_root[-1])

        # return most likely trees in order at root
        # print "\tnumber of roots to yield: "+str(len(chart[key]))  # DEBUG
        roots_yielded = 0
        while len(chart[key]) > 0 and roots_yielded < self.max_cky_trees_per_token_sequence_beam:

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
            # print "most_likely_ccg_parse_tree_given_tokens with tks="+str(tks) + \
            #  ", new_sense_leaf_limit=" + str(new_sense_leaf_limit)+", score="+str(chart[key][best_idx][3]) +\
            #  " yielding"  # DEBUG
            yield tree, chart[key][best_idx][3]  # return tree and score
            del chart[key][best_idx]
            roots_yielded += 1

        if roots_yielded == self.max_cky_trees_per_token_sequence_beam:  # DEBUG
            # print "WARNING: beam search limit hit"  # DEBUG
            pass

        # no parses left
        yield None, neg_inf

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
                raise RuntimeError("performing Merge with '"+self.print_parse(a, True) +
                                   "' taking '"+self.print_parse(a, True) +
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
            raise RuntimeError("ERROR: invalidly linked structure generated by FA: " +
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
                raise RuntimeError("WARNING: got invalidly linked node '"+self.print_parse(a)+"'")
            if not b.validate_tree_structure():  # DEBUG
                raise RuntimeError("WARNING: got invalidly linked node '"+self.print_parse(b)+"'")
        # print "performing FA with '"+self.print_parse(a, True)+"' taking '"+self.print_parse(b, True)+"'"  # DEBUG

        # if A is 'and', apply B to children
        if not a.is_lambda and self.ontology.preds[a.idx] == 'and':
            a_new = copy.deepcopy(a)
            for i in range(0, len(a.children)):
                c = a_new.children[i]
                c_obj = copy.deepcopy(self.lexicon.semantic_forms[c]) if type(c) is int else c
                copy_obj = SemanticNode.SemanticNode(None, None, None, False, 0)
                copy_obj.copy_attributes(c_obj, preserve_parent=True)
                if (copy_obj in self.lexicon.semantic_forms and
                        self.lexicon.semantic_forms.index(copy_obj) in self.type_raised):
                    copy_obj.copy_attributes(self.lexicon.semantic_forms[
                                             self.type_raised[self.lexicon.semantic_forms.index(copy_obj)]],
                                             preserve_parent=True)
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
                raise RuntimeError("ERROR: invalidly linked structure generated by FA: " +
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
                # print "substituting '"+self.print_parse(b, True)+"' for '"+self.print_parse(curr, True) + \
                #     "' with lambda offset "+str(deepest_lambda)  # DEBUG
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
                        if (c_obj in self.lexicon.semantic_forms and
                                self.lexicon.semantic_forms.index(c_obj) in self.type_raised):
                            c_obj.copy_attributes(self.lexicon.semantic_forms[
                                                  self.type_raised[self.lexicon.semantic_forms.index(c_obj)]])
                            raised = True
                        b_new.children[i] = self.perform_fa(c_obj, curr.children[0], renumerate=False)
                        b_new.children[i].increment_lambdas(inc=deepest_lambda)
                        b_new.children[i].parent = curr
                        b_new.children[i].set_return_type(self.ontology)
                    if curr.parent.is_lambda_instantiation:  # eg. 1(2) taking and(pred,pred) -> and(pred(2),pred(2)
                        b_new_arg = b_new.children[0]
                        while self.ontology.preds[b_new_arg.idx] == 'and':
                            # print "B_new_arg: "+self.print_parse(b_new_arg)  # DEBUG
                            b_new_arg = b_new_arg.children[0]
                        # print "setting curr parent lambda name to child of "+self.print_parse(b_new_arg)  # DEBUG
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
                        # print "...whole tree is instance"  # DEBUG
                        curr.copy_attributes(b)  # instance is whole tree; add nothing more and loop will now exit
                    elif b.children is None:
                        # print "...instance heads tree; preserve children taking B"
                        curr.copy_attributes(b, deepest_lambda, preserve_children=True)
                    else:
                        raise RuntimeError("Error: incompatible parentless, childed node A with childed node B")
                    entire_replacement = True
                    curr.set_category(self.lexicon.categories[a.category][0])  # take on return type of A
                    curr.set_return_type(self.ontology)
                else:
                    # print "entering standard implementation for curr"  # DEBUG
                    for curr_parent_matching_idx in range(0, len(curr.parent.children)):
                        if not curr.parent.children[curr_parent_matching_idx] != curr:  # find matching address
                            break
                    if curr.children is None:
                        # print "...instance of B ("+self.print_parse(b)+") will preserve its children"  # DEBUG
                        # lambda instance is a leaf
                        curr.parent.children[curr_parent_matching_idx].copy_attributes(b, deepest_lambda,
                                                                                       preserve_parent=True)
                        if not curr.parent.children[curr_parent_matching_idx].validate_tree_structure():  # DEBUG
                            raise RuntimeError("ERROR: copy operation produced invalidly linked tree " +
                                               self.print_parse(curr.parent.children[curr_parent_matching_idx], True))
                    else:
                        if b.children is None:
                            # print "...instance of B will keep children from A"  # DEBUG
                            curr.parent.children[curr_parent_matching_idx].copy_attributes(b, deepest_lambda,
                                                                                           preserve_parent=True,
                                                                                           preserve_children=True)
                        else:
                            # print "...instance of A and B have matching lambda headers to be merged"  # DEBUG
                            b_without_lambda_headers = b
                            lambda_types = {}
                            while (b_without_lambda_headers.is_lambda and
                                    b_without_lambda_headers.is_lambda_instantiation):
                                lambda_types[b_without_lambda_headers.lambda_name] = b_without_lambda_headers.type
                                b_without_lambda_headers = b_without_lambda_headers.children[0]
                            a_trace = a
                            lambda_map = {}
                            while len(lambda_types.keys()) > 0:
                                name_found = None
                                for name in lambda_types:
                                    if lambda_types[name] == a_trace.type:
                                        lambda_map[name] = a_trace.lambda_name
                                        name_found = name
                                        break
                                if name_found is not None:
                                    del lambda_types[name_found]
                                a_trace = a_trace.children[0]
                            # print "lambda_map: "+str(lambda_map)  # DEBUG
                            curr.parent.children[curr_parent_matching_idx].copy_attributes(b_without_lambda_headers,
                                                                                           lambda_map=lambda_map,
                                                                                           preserve_parent=True,
                                                                                           preserve_children=False)
                    curr.parent.children[curr_parent_matching_idx].set_return_type(self.ontology)
                    # print "substitution created "+self.print_parse(curr, True)  # DEBUG
            if not entire_replacement and curr.children is not None:
                to_traverse.extend([[c, deepest_lambda] for c in curr.children])
        if renumerate:
            # print "renumerating result '"+self.print_parse(ab)+"'"  # DEBUG
            ab.renumerate_lambdas([])
        try:
            ab.set_return_type(self.ontology)
        except TypeError as e:
            raise e
        ab.set_category(self.lexicon.categories[a.category][0])
        ab.commutative_raise_node(self.commutative_idxs, self.ontology)
        # print "performed FA(2) with '"+self.print_parse(a, True)+"' taking '"+self.print_parse(b, True) + \
        #       "' to form '"+self.print_parse(ab, True)+"'"  # DEBUG
        if self.safety and not ab.validate_tree_structure():
            raise RuntimeError("ERROR: invalidly linked structure generated by FA: " +
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
            if a.is_lambda_instantiation and a.children is None:  # DEBUG
                raise RuntimeError("ERROR: found lambda with no children: "+str(self.print_parse(a)))
            # print "A is not lambda instantiation or types are mismatched" + str(self.print_parse(a)) + \  # DEBUG
            #     ", "+ str(self.print_parse(b))  # DEBUG
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
        # print "performing Split with '"+self.print_parse(ab, True)+"'" #DEBUG

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

        # print "performed Split with '"+self.print_parse(ab, True)+"' to form '" + \
        #     self.print_parse(to_return[0], True)+"', '"+self.print_parse(to_return[1], True)+"'"  # DEBUG
        candidate_pairs = [[to_return[0], to_return[1]], [copy.deepcopy(to_return[1]), copy.deepcopy(to_return[0])]]
        if self.safety:
            for idx in range(0, len(candidate_pairs)):
                for jdx in range(0, len(candidate_pairs[idx])):
                    if not candidate_pairs[idx][jdx].validate_tree_structure():
                        raise RuntimeError("ERROR: invalidly linked structure generated by split: " +
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
        # print "performing reverse FA with '"+self.print_parse(a, True)+"'"  # DEBUG

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
                        # print "produced: "+self.print_parse(a1_with_cat, True)+" consuming " + \
                        #     self.print_parse(a2_with_cat, True)+" in dir "+str(d)+" with params " + \
                        #     ",".join([str(lambda_type), str(preserve_host_children), str(aa)])  # DEBUG
                        if self.safety and not a1_with_cat.validate_tree_structure():
                            raise RuntimeError("ERROR: invalidly linked structure generated by reverse FA: " +
                                               self.print_parse(a1_with_cat, True) +
                                               "with params "+",".join([str(lambda_type), str(preserve_host_children),
                                                                        str(aa)]))
                        if self.safety and not a2_with_cat.validate_tree_structure():
                            raise RuntimeError("ERROR: invalidly linked structure generated by reverse FA: " +
                                               self.print_parse(a2_with_cat, True) +
                                               "with params "+",".join([str(lambda_type), str(preserve_host_children),
                                                                        str(aa)]))

        return candidate_pairs

    # given a string, return the set of possible tokenizations of that string given lexical entries
    def tokenize(self, s):
        str_parts = s.split()
        return [str_parts]
