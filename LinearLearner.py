import math
import copy
import Action


class LinearLearner:
    def __init__(self, ont, lex, f_extractor):
        self.ontology = ont
        self.lexicon = lex
        self.feature_extractor = f_extractor
        self.theta = {}

    def updateThetaMapReturnFullFeatureMap(self, m):
        zero_m_map = copy.deepcopy(m)
        self.zeroDict(zero_m_map)
        full_feature_map = copy.deepcopy(self.theta)
        self.copyDict(zero_m_map, self.theta)  # expanded theta now zero'd at all m locations
        self.copyDict(full_feature_map,
                      self.theta)  # restore theta values at m locations; theta now expanded with 0s at new features
        self.zeroDict(full_feature_map)
        self.copyDict(m, full_feature_map)  # now full_feature_map has 0s except at m locations
        return full_feature_map

    def zeroDict(self, d):
        for k in d:
            if (type(d[k]) is not dict):
                d[k] = 0
            else:
                self.zeroDict(d[k])

    def copyDict(self, s, t):
        for k in s:
            if (type(s[k]) is not dict):
                t[k] = s[k]
            else:
                if (k not in t): t[k] = {}
                self.copyDict(s[k], t[k])

    def extractFeatureMap(self, t, p, a):
        f_map = self.feature_extractor.extract_feature_map(t, p, a)
        expanded_f_map = self.updateThetaMapReturnFullFeatureMap(f_map)
        return expanded_f_map

    def scoreFeatures(self, m, lt=None):
        if (lt == None): lt = self.theta
        s = 0
        for k in m:
            if (type(m[k]) is not dict):
                s += m[k] * lt[k]
            else:
                s += self.scoreFeatures(m[k], lt[k])
        return s

    def score_action(self, t, p, a):
        return self.scoreFeatures(self.extractFeatureMap(t, p, a))

    # probably a better way to do this that we need to think about
    def scoreParse(self, t, p):
        return self.score_action(t, p, Action.Action())

    # takes in training examples tokens, candidate parse and action, gold parse and action
    def learn_from_actions(self, t, learning_rate=None):
        if learning_rate is None: learning_rate = 1 / math.sqrt(float(len(t)))
        for [t, cp, ca, gp, ga] in t:
            f = self.extractFeatureMap(t, cp, ca)
            g = self.extractFeatureMap(t, gp, ga)
            g_zeros = copy.deepcopy(g)
            self.zeroDict(g_zeros)
            f_full = copy.deepcopy(f)
            self.copyDict(g_zeros, f_full)
            self.copyDict(f, f_full)  # in case g introduced new features f lacks
            self.updateTheta(f_full, g, learning_rate)

    # takes in training examples T=(x,y,d,y',d')
    # x is token sequence input
    # y,d is highest scoring parse/denotation pair
    # y',d' is parse/denotation of correct denotation d', if found
    def learnFromDenotations(self, T, learning_rate=None):
        if (learning_rate == None): learning_rate = 1 / math.sqrt(float(len(T)))
        for [x, y, ys, d, zy, zys, zd] in T:
            f = self.extractFeatureMap(x, y, ys, d)
            g = self.extractFeatureMap(x, zy, zys, zd)
            g_zeros = copy.deepcopy(g)
            self.zeroDict(g_zeros)
            f_full = copy.deepcopy(f)
            self.copyDict(g_zeros, f_full)
            self.copyDict(f, f_full)  # in case g introduced new features f lacks
            self.updateTheta(f_full, g, learning_rate)

    def updateTheta(self, f, g, lr, lt=None):
        if (lt == None): lt = self.theta
        #print "updating theta sub-component " + str(lt) + " from f=" + str(f) + " and g=" + str(g)  # DEBUG
        for k in lt:
            if (type(lt[k]) is not dict):
                lt[k] += lr * (g[k] - f[k])
            else:
                self.updateTheta(f[k], g[k], lr, lt[k])
