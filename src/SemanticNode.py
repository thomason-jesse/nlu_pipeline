__author__ = 'jesse'

import sys
import copy


class SemanticNode:
    def __init__(self, parent, type, category, is_lambda, idx=None, lambda_name=None, is_lambda_instantiation=None,
                 children=None):
        self.parent = parent
        self.type = type
        self.category = category
        self.is_lambda = is_lambda
        if ((self.is_lambda and (lambda_name is None or is_lambda_instantiation is None)) or
           (not self.is_lambda and idx is None)):
            sys.exit("Invalid SemanticNode instantiation (" +
                     str([self.is_lambda, lambda_name, is_lambda_instantiation, idx]) + ")")
        self.idx = idx
        self.lambda_name = lambda_name
        self.is_lambda_instantiation = is_lambda_instantiation
        self.children = children
        self.return_type = None
        self.categories_used = []
        if self.idx is not None:
            self.categories_used.append(self.idx)

    # set the category and update categories used structure
    def set_category(self, idx):
        self.category = idx
        if idx not in self.categories_used:
            self.categories_used.append(idx)

    # return own type if no children; output expected return value if children match; error out otherwise
    def set_return_type(self, ontology):
        # lambdas 'return' <type,child_return_type> if consumed by upper functions
        if self.is_lambda_instantiation:
            type_str = ontology.compose_str_from_type(self.type)
            self.children[0].set_return_type(ontology)
            child_return_type_str = ontology.compose_str_from_type(self.children[0].return_type)
            self.return_type = ontology.read_type_from_str("<" + type_str + "," + child_return_type_str + ">")
        elif self.children is None:
            self.return_type = self.type  # leaf returns itself
        else:
            candidate_type = self.type
            for c in self.children:
                c.set_return_type(ontology)
                if ontology.types[candidate_type][0] == c.return_type:
                    candidate_type = ontology.types[candidate_type][1]
                else:
                    raise TypeError("Non-matching child type " +
                                    ontology.compose_str_from_type(c.return_type) + " (" + c.print_little() +
                                    ") for parent " + ontology.compose_str_from_type(candidate_type) + " (" +
                                    self.print_little() + ")")
            self.return_type = candidate_type

    # copy attributes of the given SemanticNode into this one (essentially, clone the second into this space)
    def copy_attributes(self, a, lambda_enumeration=0, lambda_map=None, preserve_parent=False, preserve_children=False):
        self.set_category(a.category)
        self.type = a.type
        self.is_lambda = a.is_lambda
        self.idx = a.idx
        self.lambda_name = None if a.lambda_name is None else a.lambda_name + lambda_enumeration
        if lambda_map is not None and self.lambda_name in lambda_map:
            self.lambda_name = lambda_map[a.lambda_name]
        self.is_lambda_instantiation = a.is_lambda_instantiation
        if not preserve_parent:
            self.parent = a.parent
        if not preserve_children:
            if a.children is None:
                self.children = None
            else:
                self.children = [SemanticNode(self, 0, 0, False, 0) for _ in range(0, len(a.children))]
                for i in range(0, len(a.children)):
                    self.children[i].copy_attributes(a.children[i], lambda_enumeration=lambda_enumeration,
                                                     lambda_map=lambda_map, preserve_parent=True)
        self.return_type = a.return_type

    def print_little(self):
        return "(" + ",".join(
            [str(self.is_lambda), str(self.type), str(self.lambda_name) if self.is_lambda else str(self.idx)]) + ")"

    def __str__(self):
        return str(self.__key())

    def renumerate_lambdas(self, lambdas):
        if self.is_lambda:
            if self.is_lambda_instantiation:
                lambdas.append(self.lambda_name)
                self.lambda_name = len(lambdas)
            else:
                self.lambda_name = lambdas.index(self.lambda_name) + 1
        if self.children is not None:
            for c in self.children:
                c.renumerate_lambdas(lambdas[:])

    # validate parent/child relationships
    def validate_tree_structure(self):
        if self.children is None:
            return True
        for idx in range(0, len(self.children)):
            if self.children[idx].parent != self:
                print "child "+str(idx)+" of "+str(self.children[idx]) + \
                      " has non-matching parent "+str(self.children[idx].parent)  # DEBUG
                return False
            if not self.children[idx].validate_tree_structure():
                return False
        return True

    # increment lambda values by given num
    def increment_lambdas(self, inc=1):
        if self.is_lambda:
            self.lambda_name += inc
        if self.children is not None:
            for c in self.children:
                c.increment_lambdas(inc)

    # these are a forward comparisons, so two trees are considered equal if their roots have different parents,
    # as long as the roots and children down are identical categories

    def equal_allowing_commutativity(self, other, commutative_idxs, ignore_syntax=True, ontology=None):
        a = copy.deepcopy(self)
        b = copy.deepcopy(other)
        a.commutative_raise_node(commutative_idxs, ontology=ontology)
        b.commutative_raise_node(commutative_idxs, ontology=ontology)
        return a.equal_ignoring_syntax(b, ignore_syntax=ignore_syntax)

    def commutative_raise_node(self, commutative_idxs, ontology=None):  # support function
        to_expand = [self]
        while len(to_expand) > 0:
            curr = to_expand.pop()
            if curr.idx in commutative_idxs:
                new_c = []
                if curr.children is not None:
                    for c in curr.children:
                        new_c.extend(self.commutative_raise(c, curr.idx))
                    for nc in new_c:
                        nc.parent = curr
                    new_c = sorted(new_c, key=lambda node: node.idx)
                    curr.children = new_c
                    if ontology is not None:
                        curr.set_type_from_children_return_types(curr.children[0].return_type, ontology)
            elif curr.children is not None:
                to_expand.extend(curr.children)

    # given children, add types as necessary to make func take them and return r
    def set_type_from_children_return_types(self, r, ontology):
        new_type = r
        for idx in range(0, len(self.children)):
            new_type_form = [self.children[idx].return_type, new_type]
            if new_type_form not in ontology.types:
                ontology.types.append(new_type_form)
            new_type = ontology.types.index(new_type_form)
        self.type = new_type

    def commutative_raise(self, node, idx):  # support function
        if node.idx == idx and node.children is not None:
            new_c = []
            for c in node.children:
                new_c.extend(self.commutative_raise(c, idx))
            return new_c
        else:
            return [node]

    def equal_ignoring_syntax(self, other, ignore_syntax=True):
        if type(self) != type(other):
            # print "type mismatch"  # DEBUG
            return False  # ie no casting
        if (self.type == other.type and self.is_lambda == other.is_lambda and self.idx == other.idx and
                self.lambda_name == other.lambda_name and
                (ignore_syntax or self.category == other.category)):
            if self.children is None and other.children is None:
                return True
            if ((self.children is None and other.children is not None) or
                    (self.children is not None and other.children is None)):
                # print "presence of children mismatch"  # DEBUG
                return False
            if len(self.children) == len(other.children):
                for i in range(0, len(self.children)):
                    if not (self.children[i].equal_ignoring_syntax(other.children[i], ignore_syntax=ignore_syntax)):
                        # print "child mismatch at idx "+str(i)  # DEBUG
                        return False
                return True
            else:
                # print "size of children vectors mismatch"  # DEBUG
                return False
        else:
            # print "type, lambda, idx, or lambda name mismatch"  # DEBUG
            return False

    def __eq__(self, other):
        if not self.equal_ignoring_syntax(other, ignore_syntax=False):
            return False
        return True

    # implementations of __key and __hash__ map two nodes to the same value if they and their children share all
    # identifying attributes, but they need not be the same object ID nor share a parent
    def __key(self):
        c_keys = []
        if self.children is not None:
            for c in self.children:
                c_keys.append(c.__hash__())
        key = (self.type, self.category, self.is_lambda, self.idx, self.lambda_name, self.is_lambda_instantiation,
               tuple(c_keys))
        return key

    def __hash__(self):
        return hash(self.__key())
