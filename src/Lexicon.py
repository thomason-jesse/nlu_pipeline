__author__ = 'jesse'

import sys
import SemanticNode


class Lexicon:
    def __init__(self, ontology, lexicon_fname, allow_expanding_ont=False):
        self.ontology = ontology
        self.categories = []  # will grow on its own
        self.surface_forms, self.semantic_forms, self.entries, self.pred_to_surface = self.read_lex_from_file(
            lexicon_fname, allow_expanding_ont)
        self.reverse_entries = []
        self.sem_form_expected_args = None
        self.sem_form_return_cat = None
        self.category_consumes = None
        self.generator_should_flush = False
        self.update_support_structures()

    def update_support_structures(self):
        self.compute_pred_to_surface(self.pred_to_surface)
        self.reverse_entries = self.compute_reverse_entries()
        self.sem_form_expected_args = [self.calc_exp_args(i) for i in range(0, len(self.semantic_forms))]
        self.sem_form_return_cat = [self.calc_return_cat(i) for i in range(0, len(self.semantic_forms))]
        self.category_consumes = [self.find_consumables_for_cat(i) for i in range(0, len(self.categories))]
        self.generator_should_flush = True

    def compute_pred_to_surface(self, pts):
        for sur_idx in range(0, len(self.entries)):
            for sem_idx in self.entries[sur_idx]:
                to_examine = [self.semantic_forms[sem_idx]]
                while len(to_examine) > 0:
                    curr = to_examine.pop()
                    if not curr.is_lambda:
                        if curr.idx in pts and sur_idx not in pts[curr.idx]:
                            pts[curr.idx].append(sur_idx)
                        elif curr.idx not in pts:
                            pts[curr.idx] = [sur_idx]
                    if curr.children is not None:
                        to_examine.extend(curr.children)

    def compute_reverse_entries(self):
        r = {}
        for sur_idx in range(0, len(self.surface_forms)):
            for sem_idx in self.entries[sur_idx]:
                if sem_idx in r and sur_idx not in r[sem_idx]:
                    r[sem_idx].append(sur_idx)
                else:
                    r[sem_idx] = [sur_idx]
        for sem_idx in range(0, len(self.semantic_forms)):
            if sem_idx not in r:
                r[sem_idx] = []
        r_list = []
        for i in range(0, len(r)):
            if i in r:
                r_list.append(r[i])
            else:
                r_list.append([])
        return r_list

    def calc_exp_args(self, idx):
        exp_args = 0
        curr_cat = self.semantic_forms[idx].category
        while type(self.categories[curr_cat]) is list:
            exp_args += 1
            curr_cat = self.categories[curr_cat][0]
        return exp_args

    def calc_return_cat(self, idx):
        curr_cat = self.semantic_forms[idx].category
        while type(self.categories[curr_cat]) is list:
            curr_cat = self.categories[curr_cat][0]
        return curr_cat

    def find_consumables_for_cat(self, idx):
        consumables = []
        for sem_form in self.semantic_forms:
            curr = self.categories[sem_form.category]
            while type(curr) is list and type(self.categories[curr[0]]):
                if curr[0] == idx:
                    break
                curr = self.categories[curr[0]]
            if curr[0] == idx:
                cons = [curr[1], curr[2]]
                if cons not in consumables:
                    consumables.append(cons)  # store both direction and consumable category
        return consumables

    def get_or_add_category(self, c):
        if c in self.categories:
            return self.categories.index(c)
        self.categories.append(c)
        return len(self.categories) - 1

    def compose_str_from_category(self, idx):
        if idx is None:
            return "NONE IDX"
        if type(self.categories[idx]) is str:
            return self.categories[idx]
        s = self.compose_str_from_category(self.categories[idx][0])
        if type(self.categories[self.categories[idx][0]]) is not str: s = '(' + s + ')'
        if self.categories[idx][1] == 0:
            s += '\\'
        else:
            s += '/'
        s2 = self.compose_str_from_category(self.categories[idx][2])
        if type(self.categories[self.categories[idx][2]]) is not str: s2 = '(' + s2 + ')'
        return s + s2

    def get_semantic_forms_for_surface_form(self, surface_form):
        if surface_form not in self.surface_forms:
            return []
        else:
            return self.entries[self.surface_forms.index(surface_form)]

    def get_surface_forms_for_predicate(self, pred):
        if type(pred) is str:
            if pred in self.ontology.preds:
                return self.pred_to_surface[self.ontology.preds.index(pred)]
            else:
                return []
        else:
            if pred in self.pred_to_surface:
                return self.pred_to_surface[pred]
            else:
                return []

    def read_lex_from_file(self, fname, allow_expanding_ont):
        surface_forms = []
        semantic_forms = []
        entries = []
        pred_to_surface = {}
        f = open(fname, 'r')
        lines = f.readlines()
        self.expand_lex_from_strs(lines, surface_forms, semantic_forms, entries, pred_to_surface,
                                  allow_expanding_ont=allow_expanding_ont)
        f.close()
        return surface_forms, semantic_forms, entries, pred_to_surface

    def expand_lex_from_strs(
            self, lines, surface_forms, semantic_forms, entries, pred_to_surface, allow_expanding_ont=False):
        for line_idx in range(0, len(lines)):
            line = lines[line_idx]

            # skip blank and commented lines
            line = line.strip()
            if len(line) == 0 or line[0] == '#':
                continue

                # add lexical entry
            lhs, rhs = line.split(" :- ")
            surface_form = lhs.strip()
            try:
                sur_idx = surface_forms.index(surface_form)
            except ValueError:
                sur_idx = len(surface_forms)
                surface_forms.append(surface_form)
                entries.append([])
                
            cat_idx, semantic_form = self.read_syn_sem(rhs, allow_expanding_ont=allow_expanding_ont)
            try:
                sem_idx = semantic_forms.index(semantic_form)
            except ValueError:
                sem_idx = len(semantic_forms)
                semantic_forms.append(semantic_form)
            entries[sur_idx].append(sem_idx)
            preds_in_semantic_form = self.get_all_preds_from_semantic_form(semantic_forms[sem_idx])
            for pred in preds_in_semantic_form:
                if pred in pred_to_surface:
                    pred_to_surface[pred].append(sur_idx)
                else:
                    pred_to_surface[pred] = [sur_idx]

    def read_syn_sem(self, s, allow_expanding_ont=False):
        lhs, rhs = s.split(" : ")
        cat_idx = self.read_category_from_str(lhs.strip())
        semantic_form = self.read_semantic_form_from_str(rhs.strip(), cat_idx, None, [], allow_expanding_ont)
        return cat_idx, semantic_form

    def get_all_preds_from_semantic_form(self, node):
        node_preds = []
        if not node.is_lambda: node_preds.append(node.idx)
        if node.children is None: return node_preds
        for c in node.children:
            node_preds.extend(self.get_all_preds_from_semantic_form(c))
        return node_preds

    def form_contains_DESC_predicate(self, node):
        if not node.is_lambda:
            for sem in self.semantic_forms:
                if not sem.is_lambda and sem.idx == node.idx and sem.category == self.categories.index('DESC'):
                    return True
        if node.children is not None:
            for c in node.children:
                if self.form_contains_DESC_predicate(c):
                    return True
        return False

    def read_category_from_str(self, s):
        # detect whether (,) surrounding s and pop it out of them
        if s[0] == '(':
            p = 1
            for i in range(1, len(s) - 1):
                if s[i] == '(':
                    p += 1
                elif s[i] == ')':
                    p -= 1
                if p == 0: break
            if i == len(s) - 2 and p == 1 and s[-1] == ')': s = s[1:-1]

            # everything up to final unbound /,\ is output, category following is input
        p = 0
        fin_slash_idx = len(s) - 1
        while fin_slash_idx >= 0:
            if s[fin_slash_idx] == ')':
                p += 1
            elif s[fin_slash_idx] == '(':
                p -= 1
            elif p == 0:
                if s[fin_slash_idx] == '/':
                    direction = 1
                    break
                elif s[fin_slash_idx] == '\\':
                    direction = 0
                    break
            fin_slash_idx -= 1
        if fin_slash_idx > 0:  # input/output pair is being described
            output_category_idx = self.read_category_from_str(s[:fin_slash_idx])
            input_category_idx = self.read_category_from_str(s[fin_slash_idx + 1:])
            category = [output_category_idx, direction, input_category_idx]
        else:  # this is an atomic category
            if '(' in s or ')' in s or '/' in s or '\\' in s: sys.exit("Invalid atomic category '" + s + "'")
            category = s
        try:
            idx = self.categories.index(category)
        except ValueError:
            idx = len(self.categories)
            self.categories.append(category)
        return idx

    def read_semantic_form_from_str(self, s, category, parent, scoped_lambdas, allow_expanding_ont):
        # the node to be instantiated and returned
        s = s.strip()

        # if head is lambda, create unary lambda node branched from root
        if s[:6] == "lambda":
            str_parts = s[6:].strip().split('.')
            info = str_parts[0]
            name, type_str = info.split(':')
            scoped_lambdas.append(name)
            name_idx = len(scoped_lambdas)
            t = self.ontology.read_type_from_str(type_str)
            node = SemanticNode.SemanticNode(parent, t, category, True, lambda_name=name_idx,
                                             is_lambda_instantiation=True)
            str_remaining = '.'.join(str_parts[1:])  # remove lambda prefix
            str_remaining = str_remaining[1:-1]  # remove scoping parens that follow lambda declaration

            # gather head (either atomic or a predicate with following arguments)
        else:
            end_of_pred = 1
            while end_of_pred < len(s):
                if s[end_of_pred] == '(':
                    break
                end_of_pred += 1
            pred = s[:end_of_pred]

            # check whether head is a lambda name
            curr = parent
            is_scoped_lambda = False
            while curr is not None and not is_scoped_lambda:
                try:
                    pred_idx = scoped_lambdas.index(pred) + 1
                except ValueError:
                    pred_idx = None
                is_scoped_lambda = True if (curr.is_lambda and curr.lambda_name == pred_idx) else False
                if is_scoped_lambda: break
                curr = curr.parent
            if is_scoped_lambda:
                node = SemanticNode.SemanticNode(parent, curr.type, None, True, lambda_name=curr.lambda_name,
                                                 is_lambda_instantiation=False)

                # else, head is an ontological predicate
            else:
                try:
                    pred_idx = self.ontology.preds.index(pred)
                except ValueError:
                    if not allow_expanding_ont:
                        sys.exit("Symbol not found within ontology or lambdas in scope: '" + pred + "'")
                    else:
                        # assume unknown predicates are of type <e,t> with entry pred :- DESC : pred
                        pred_idx = len(self.ontology.preds)
                        self.ontology.preds.append(pred)
                        e_to_t_idx = self.ontology.types.index(
                            [self.ontology.types.index('e'), self.ontology.types.index('t')])
                        self.ontology.entries.append(e_to_t_idx)
                        self.ontology.num_args.append(self.ontology.calc_num_pred_args(pred_idx))
                        self.surface_forms.append(pred)
                        self.semantic_forms.append(SemanticNode.SemanticNode(
                            None, e_to_t_idx, self.categories.index('DESC'), False, idx=pred_idx))
                        self.entries.append([len(self.semantic_forms)-1])
                        self.update_support_structures()
                node = SemanticNode.SemanticNode(parent, self.ontology.entries[pred_idx], category, False, idx=pred_idx)

            # remove scoping parens that enclose argument(s) to predicate (if atomic named lambda, may be empty)
            str_remaining = s[end_of_pred + 1:-1]
            # find strings defining children and add recursively
        if len(str_remaining) > 0:
            delineating_comma_idxs = []
            p = d = 0
            for i in range(0, len(str_remaining)):
                if str_remaining[i] == '(':
                    p += 1
                elif str_remaining[i] == ')':
                    p -= 1
                elif str_remaining[i] == '<':
                    d += 1
                elif str_remaining[i] == '>':
                    d -= 1
                elif str_remaining[i] == ',' and p == 0 and d == 0:
                    delineating_comma_idxs.append(i)
            children = []
            splits = [-1]
            splits.extend(delineating_comma_idxs)
            splits.append(len(str_remaining))
            expected_child_cats = []
            curr_cat = category
            while curr_cat is not None and type(self.categories[curr_cat]) is list:
                curr_cat = self.categories[curr_cat][0]
                expected_child_cats.append(curr_cat)
            for i in range(1, len(splits)):
                e_cat = expected_child_cats[i - 1] if len(expected_child_cats) >= i else None
                children.append(
                    self.read_semantic_form_from_str(
                        str_remaining[splits[i-1] + 1:splits[i]], e_cat, node, scoped_lambdas[:], allow_expanding_ont))
            node.children = children
        try:
            node.set_return_type(self.ontology)
        except TypeError as e:
            print e
            sys.exit("Offending string: '" + s + "'")

        if not node.validate_tree_structure():
            sys.exit("ERROR: read in invalidly linked semantic node from string '"+s+"'")  # DEBUG
        return node

    def delete_semantic_form_for_surface_form(self, surface_form, ont_idx) :
        if surface_form not in self.surface_forms :
            return
        matching_semantic_form = None
        for semantic_form in self.semantic_forms :
            if semantic_form.idx == ont_idx :
                matching_semantic_form = semantic_form
                break
        if matching_semantic_form is None :
            return
            
        sur_idx = self.surface_forms.index(surface_form)
        sem_idx = self.semantic_forms.index(matching_semantic_form)
        
        if sur_idx in self.entries :
            if sem_idx in self.entries[sur_idx] :
                self.entries[sur_idx].remove(sem_idx)
                
        if ont_idx in self.pred_to_surface :
            if sur_idx in self.pred_to_surface[ont_idx] :
                self.pred_to_surface.remove(sur_idx)
                
        if sem_idx in self.reverse_entries :
            if sur_idx in self.reverse_entries[sem_idx] :
                self.reverse_entries.remove(sur_idx)
        
