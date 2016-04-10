__author__ = 'jesse'

import sys


class Ontology:
    # initializes an ontology data structure which reads in atoms and predicates from given files
    def __init__(self, ont_fname):

        # subsequent entries are tuples of indices into this list defining a binary hierarchy
        self.types = ['t', 'e', 'd', 'a', 'c']

        # get predicates and map from predicates to types
        self.preds, self.entries = self.read_sem_from_file(ont_fname)

        # set special UNK predicates the parser can assign to for parsing with unknowns in synonym detection
        self.preds.append("UNK_E")
        self.entries.append(self.types.index('e'))

        # calculate and store number of arguments each predicate takes (atoms take 0)
        self.num_args = [self.calc_num_pred_args(i) for i in range(0, len(self.preds))]

    # calculate the number of arguments a predicate takes
    def calc_num_pred_args(self, idx):
        num_args = 0
        curr_type = self.types[self.entries[idx]]
        while type(curr_type) is list:
            num_args += 1
            curr_type = self.types[curr_type[1]]
        return num_args

    # reads semantic atom/predicate declarations from a given file of format:
    # atom_name:type
    # pred_name:<complex_type>
    def read_sem_from_file(self, fname):

        preds = []
        entries = []  # map of pred_idx:type read in
        f = open(fname, 'r')
        for line in f.readlines():

            # ignore blank lines and comments
            line = line.strip()
            if len(line) == 0 or line[0] == '#':
                continue

            # create semantic meaning representation from string
            [name, type_str] = line.split(':')
            if name in preds: sys.exit("Multiply defined type for predicate '" + name + "'")
            entries.append(self.read_type_from_str(type_str))
            preds.append(name)
        f.close()
        return preds, entries

    # returns the index of self.types at which this type is stored; adds types to this list as necessary to compose such a type
    def read_type_from_str(self, str):

        # a complex type
        if str[0] == "<" and str[-1] == ">":
            d = 0
            for split_idx in range(1, len(str) - 1):
                if str[split_idx] == '<':
                    d += 1
                elif str[split_idx] == '>':
                    d -= 1
                elif str[split_idx] == ',' and d == 0:
                    break
            comp_type = [self.read_type_from_str(str[1:split_idx]), self.read_type_from_str(str[split_idx + 1:-1])]
            try:
                return self.types.index(comp_type)
            except ValueError:
                self.types.append(comp_type)
                return len(self.types) - 1

        # a primitive type
        else:
            try:
                return self.types.index(str)
            except ValueError:
                sys.exit("Unrecognized primitive type '" + str + "'")

    # returns a string representing the given ontological type
    def compose_str_from_type(self, t):
        s = ''
        # a complex type
        if type(self.types[t]) is list:
            s += '<' + self.compose_str_from_type(self.types[t][0]) + ',' + self.compose_str_from_type(self.types[t][1]) + '>'
        # a primitive type
        else:
            s += self.types[t]
        return s
