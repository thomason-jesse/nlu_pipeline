__author__ = 'aishwarya'

import pickle

# For typechecking
from SemanticNode import SemanticNode

def save_model(obj, name):
    with open('src/nlu_pipeline/src/models/'+ str(name) + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_model(name):
    try :
        with open('src/nlu_pipeline/src/models/' + str(name) + '.pkl', 'r') as f:
            return pickle.load(f)
    except :
        return None
        
def checkLists(list1, list2) :
    if list1 == None and list2 == None :
        return True
    elif list1 == None and not list2 == None :
        return False
    elif not list1 == None and list2 == None :
        return False
    else :
        if type(list1) != list or type(list2) != list :
            return False
        elif set(list1) == set(list2) :
            return True
        else :
            return False
            
def checkDicts(dict1, dict2) :
    if dict1 == None and dict2 == None :
        return True
    elif dict1 == None and not dict2 == None :
        return False
    elif not dict1 == None and dict2 == None :
        return False
    else :
        if type(dict1) != dict or type(dict2) != dict :
            return False
        if dict1.keys() != dict2.keys() :
            return False   
        else :
            for key in dict1.keys() :
                if type(dict1[key]) == list and not checkLists(dict1[key], dict2[key]) :
                    return False
                elif not dict1[key] == dict2[key] :
                    return False
    return True

def arg_max(d):
    max = None
    arg_max = None
    for k in d:
        if max is None or max < d[k]:
            max = d[k]
            arg_max = k
    return arg_max, max
    
def predicate_holds(predicate, argument, grounder) :
    if argument is None :
        return False
    if argument not in grounder.ontology.preds :
        return False
    if predicate not in grounder.ontology.preds :
        return False    
    arg_idx = grounder.ontology.preds.index(argument)
    pred_idx = grounder.ontology.preds.index(predicate)
    child = SemanticNode(None, 9, 15, False, arg_idx)
    parent = SemanticNode(None, 9, 15, False, pred_idx, children=[child])
    child.parent = parent  
    g = grounder.groundSemanticNode(parent, [], [], [])
    answers = grounder.grounding_to_answer_set(g)
    if True in answers :
        return True
    else :
        return False

