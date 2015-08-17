import sys


class SemanticNode:
    def __init__(self, parent, type, category, is_lambda, idx=None, lambda_name=None, is_lambda_instantiation=None,
                 children=None):
        self.parent = parent
        self.type = type
        self.category = category
        self.is_lambda = is_lambda
        if ((self.is_lambda and (lambda_name is None or is_lambda_instantiation is None)) or
            (not self.is_lambda and idx is None)):
                sys.exit("Invalid SemanticNode instantiation (" + str([self.is_lambda, lambda_name, is_lambda_instantiation, idx]) + ")")
        self.idx = idx
        self.lambda_name = lambda_name
        self.is_lambda_instantiation = is_lambda_instantiation
        self.children = children
        self.return_type = None

    # return own type if no children; output expected return value if children match; error out otherwise
    def set_return_type(self, ontology):
        # lambdas 'return' <type,child_return_type> if consumed by upper functions
        if self.is_lambda_instantiation:
            type_str = ontology.compose_str_from_type(self.type)
            child_return_type_str = ontology.compose_str_from_type(self.children[0].return_type)
            self.return_type = ontology.read_type_from_str("<" + type_str + "," + child_return_type_str + ">")
        elif self.children is None:
            self.return_type = self.type  # leaf returns itself
        else:
            candidate_type = self.type
            for c in self.children:
                if c.return_type is None: c.set_return_type(ontology)
                if ontology.types[candidate_type][0] == c.return_type:
                    candidate_type = ontology.types[candidate_type][1]
                else:
                    raise TypeError("Non-matching child type " + str(
                        ontology.types[c.return_type]) + " (" + c.print_little() + ") for parent " + str(
                        ontology.types[candidate_type]) + " (" + self.print_little() + ")")
            self.return_type = candidate_type

    # copy attributes of the given SemanticNode into this one (essentially, clone the second into this space)
    def copy_attributes(self, A, lambda_enumeration=0, preserve_parent=False, preserve_children=False):
        self.category = A.category
        self.type = A.type
        self.is_lambda = A.is_lambda
        self.idx = A.idx
        self.lambda_name = None if A.lambda_name is None else A.lambda_name + lambda_enumeration
        self.is_lambda_instantiation = A.is_lambda_instantiation
        if not preserve_parent: self.parent = A.parent
        if not preserve_children:
            if A.children is None:
                self.children = None
            else:
                self.children = [SemanticNode(self, 0, 0, False, 0) for i in range(0, len(A.children))]
                for i in range(0, len(A.children)): self.children[i].copy_attributes(A.children[i], lambda_enumeration)
        self.return_type = A.return_type

    def print_little(self):
        return "(" + ",".join(
            [str(self.is_lambda), str(self.type), str(self.lambda_name) if self.is_lambda else str(self.idx)]) + ")"

    def __str__(self):
        s = "(" + ",".join(
            [str(self.is_lambda), str(self.type), str(self.lambda_name) if self.is_lambda else str(self.idx)]) + ")"
        if self.children is not None:
            for c in self.children:
                s += "\n"
                curr = self
                while curr is not None:
                    s += "\t"
                    curr = curr.parent
                s += str(c)
        return s

    # this is a forward comparison, so two trees are considered equal if their roots have different parents,
    # as long as the roots and children down are identical categories need not match since this is a test
    # for semantic, not syntactic, equality
    def __eq__(self, other):
        if type(self) != type(other): return False  # ie no casting
        if (self.type == other.type and self.is_lambda == other.is_lambda and self.idx == other.idx and
                self.lambda_name == other.lambda_name):
            if self.children is None and other.children is None: return True
            if self.children is None and other.children is not None: return False
            if self.children is not None and other.children is None: return False
            if len(self.children) == len(other.children):
                for i in range(0, len(self.children)):
                    if not (self.children[i] == other.children[i]): return False
                return True
            else:
                return False
        else:
            return False
