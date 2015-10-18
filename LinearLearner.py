__author__ = 'jesse'

import math
import copy
import Action


class LinearLearner:
    def __init__(self, ont, lex, f_extractor):
        self.ontology = ont
        self.lexicon = lex
        self.feature_extractor = f_extractor
        self.theta = {}

    def update_theta_map_return_full_feature_map(self, m):
        zero_m_map = copy.deepcopy(m)
        self.zero_dict(zero_m_map)
        full_feature_map = copy.deepcopy(self.theta)
        self.copy_dict(zero_m_map, self.theta)  # expanded theta now zero'd at all m locations
        self.copy_dict(full_feature_map,
                       self.theta)  # restore theta values at m locations; theta now expanded with 0s at new features
        self.zero_dict(full_feature_map)
        self.copy_dict(m, full_feature_map)  # now full_feature_map has 0s except at m locations
        return full_feature_map

    def zero_dict(self, d):
        for k in d:
            if type(d[k]) is not dict:
                d[k] = 0
            else:
                self.zero_dict(d[k])

    def copy_dict(self, s, t):
        for k in s:
            if type(s[k]) is not dict:
                t[k] = s[k]
            else:
                if k not in t:
                    t[k] = {}
                self.copy_dict(s[k], t[k])

    def extract_feature_map(self, t, p, a):
        f_map = self.feature_extractor.extract_feature_map(t, p, a)
        expanded_f_map = self.update_theta_map_return_full_feature_map(f_map)
        return expanded_f_map

    def score_features(self, m, lt=None):
        if lt is None:
            lt = self.theta
        s = 0
        for k in m:
            if type(m[k]) is not dict:
                s += m[k] * lt[k]
            else:
                s += self.score_features(m[k], lt[k])
        return s

    def score_action(self, t, p, a):
        return self.score_features(self.extract_feature_map(t, p, a))

    # probably a better way to do this that we need to think about
    def score_parse(self, t, p):
        return self.score_action(t, p, Action.Action())

    # takes in training examples candidate/gold tokens, candidate parse, and action
    def learn_from_actions(self, t, learning_rate=None):
        if len(t) == 0:
            return
        if learning_rate is None:
            learning_rate = 1 / math.sqrt(float(len(t)))
        for [ct, cp, ca, gt, gp, ga] in t:
            f = self.extract_feature_map(ct, cp, ca)
            g = self.extract_feature_map(gt, gp, ga)
            g_zeros = copy.deepcopy(g)
            self.zero_dict(g_zeros)
            f_full = copy.deepcopy(f)
            self.copy_dict(g_zeros, f_full)
            self.copy_dict(f, f_full)  # in case g introduced new features f lacks
            self.update_theta(f_full, g, learning_rate)

    def update_theta(self, f, g, lr, lt=None):
        if lt is None:
            lt = self.theta
        # print "updating theta sub-component " + str(lt) + " from f=" + str(f) + " and g=" + str(g)  # DEBUG
        for k in lt:
            if type(lt[k]) is not dict:
                lt[k] += lr * (g[k] - f[k])
            else:
                self.update_theta(f[k], g[k], lr, lt[k])
