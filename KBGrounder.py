#!/usr/bin/env python

import sys
import rospy
import copy
from bwi_kr_execution.srv import *
from bwi_kr_execution.msg import *


class KBGrounder:
    def __init__(self, ontology):
        self.ontology = ontology
        pass

    def static_fact_query_client(self, req):
        rospy.wait_for_service('static_fact_query')
        try:
            static_fact_query = rospy.ServiceProxy('static_fact_query', StaticFactQuery)
            res = static_fact_query(req)
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    # take in a semantic grounding list and return an answer set
    def grounding_to_answer_set(self, g):
        if type(g) is list:
            if len(g) == 0: return []
            if type(g[0]) is str: return g
            return [gr[1] for gr in g]
        else: return g

    # returns possible groundings for given semantic node
    def groundSemanticNode(self, root, lambda_names, lambda_types, lambda_assignments):
        groundings = []
        # print "grounding "+str(root)+" with "+str(lambda_names)+","+str(lambda_types)+","+str(lambda_assignments) #DEBUG

        # if lambda, index and try assignments on children
        if root.is_lambda and root.is_lambda_instantiation:
            new_names = lambda_names[:]
            new_names.append(root.lambda_name)
            new_types = lambda_types[:]
            new_types.append(root.type)
            for assignment in self.assignmentsForType(root.type):
                new_assignments = lambda_assignments[:]
                new_assignments.append(assignment)
                grounding_results = self.groundSemanticNode(root.children[0], new_names, new_types, new_assignments)
                for gr in grounding_results:
                    if gr[1] != False:  # satisfied with this assignment, so pass up
                        groundings.append(gr)
            # print "lambda groundings: "+str(groundings)  # DEBUG
            return groundings

        # if lambda instance, make given assignment and return
        elif root.is_lambda and not root.is_lambda_instantiation:
            if root.children is None:  # lambda leaf
                return [self.ontology.preds[lambda_assignments[lambda_names.index(root.lambda_name)]]]
            else:  # lambda predicate
                replaced = copy.deepcopy(root)
                replaced.is_lambda = False
                replaced.is_lambda_instantiation = False
                replaced.lambda_name = None
                replaced.idx = lambda_assignments[lambda_names.index(root.lambda_name)]
                # print "grounding lambda predicate instance as "+self.ontology.preds[replaced.idx]
                return self.groundSemanticNode(replaced, lambda_names, lambda_types, lambda_assignments)

        # leaf predicate/atom
        elif root.children is None:
            return [self.ontology.preds[root.idx]]

        # if type is predicate, ground arguments and evaluate
        else:
            child_grounds = []
            for c in root.children:
                child_grounds.append(self.groundSemanticNode(c, lambda_names, lambda_types, lambda_assignments))
            # print self.ontology.preds[root.idx]+" child grounds: "+str(child_grounds) #DEBUG
            # for every combination of child groundings we resolve and return a different grounding
            child_ground_idx = [0 for i in range(0, len(child_grounds))]
            while True:

                if min([len(cg) for cg in child_grounds]) == 0: break  # unsatisfiable child(ren)

                # if special predicate, handle here
                if self.ontology.preds[root.idx] == 'and':
                    satisfied = child_grounds[i][child_ground_idx[i]][1]
                    for i in range(0, len(root.children)):
                        if not child_grounds[i][child_ground_idx[i]][1]:
                            satisfied = False
                            break
                elif self.ontology.preds[root.idx] == 'or':
                    satisfied = False
                    for i in range(0, len(root.children)):
                        if child_grounds[i][child_ground_idx[i]][1] != False:
                            satisfied = child_grounds[i][child_ground_idx[i]][1]
                            break
                elif self.ontology.preds[root.idx] == 'the':
                    # print "'the' child grounds to inspect: " + str(child_grounds[0][child_ground_idx[0]])  # DEBUG
                    if len(child_grounds[0]) == 1:
                        # set satisfied to the satisfying lambda arg that heads child
                        satisfied = child_grounds[0][child_ground_idx[0]][0][len(lambda_assignments)]
                    else:
                        satisfied = False
                elif self.ontology.preds[root.idx] == 'a':
                    # print "'a' child grounds to inspect: " + str(child_grounds[0][child_ground_idx[0]])  # DEBUG
                    if len(child_grounds[0]) > 0:
                        # set satisfies, so choose arbitrary element to return (here, first)
                        satisfied = child_grounds[0][child_ground_idx[0]][0][len(lambda_assignments)]
                    else:
                        satisfied = False

                # if KB predicate, query ROS
                else:
                    # print "detected KB predicate "+self.ontology.preds[root.idx]  # DEBUG
                    req = StaticFactQueryRequest()
                    a = AspAtom()
                    a.name = self.ontology.preds[root.idx]
                    a.params = []
                    for i in range(0, len(root.children)):
                        # take the first grounding result from the set if it is multi-element
                        a.params.append(self.grounding_to_answer_set(child_grounds[i])[0])
                    req.facts.append(a)
                    try:
                        res = self.static_fact_query_client(req)
                        # print "KB request: "+str(req)+"\nsatisfied: "+str(res.satisfied)  # DEBUG
                    except genpy.message.SerializationError:
                        sys.exit("Invoked ROS KB with unknown predicate or parameters: \n" + str(req))
                        # + "\n"+"\n".join([str(child_grounds), str(child_ground_idx), str(root.children)]))
                    satisfied = res.satisfied

                # add to groundings if successful
                if satisfied != False:
                    groundings.append([[self.ontology.preds[i] for i in lambda_assignments], satisfied])

                    # 'a' predicate has a restriction to return only a singleton, so exit as soon as something is added
                    if self.ontology.preds[root.idx] == 'a':
                        break

                # iterate idx counter to try new grounding terms
                max_idx = [0 for i in range(0, len(child_ground_idx))]
                for i in range(0, len(child_ground_idx)):
                    child_ground_idx[i] += 1
                    if child_ground_idx[i] == len(child_grounds[i]):
                        child_ground_idx[i] = 0
                        max_idx[i] = 1
                    else:
                        break
                if sum(max_idx) == len(root.children): break
            # print "groundings (root "+self.ontology.preds[root.idx]+"): "+str(groundings)  # DEBUG
            return groundings

    # returns all possible ontological assignments to lambdas of a given type
    def assignmentsForType(self, type):
        return [i for i in range(0, len(self.ontology.preds)) if self.ontology.entries[i] == type]
