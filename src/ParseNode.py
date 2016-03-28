__author__ = 'jesse'

import sys
import copy


# these are used to represent nodes in a parse tree
# each has a semantic node member, which carries the semantic meaning at this point in the tree
# the parent and children relationships are to other ParseNodes, differentiating these from
# SemanticNodes, where the parent and children relationships are to parse of the semantic expression itself
class ParseNode:
    def __init__(self, parent, node, surface_form=None, semantic_form=None,  children=None):
        self.parent = parent
        self.node = node
        self.surface_form = surface_form
        self.semantic_form = semantic_form
        self.children = children

    # do a depth-first trace of the tree structure to build up and return a list of leaf nodes
    def get_leaves(self):
        if self.children is not None:
            leaves = self.children[0].get_leaves()
            leaves.extend(self.children[1].get_leaves())
            return leaves
        else:
            return [self]