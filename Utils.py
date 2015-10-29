__author__ = 'aishwarya'

import pickle

def save_model(obj, name):
    with open('src/bwi_common/bwi_dialog/src/models/'+ str(name) + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_model(name):
    try :
        with open('src/bwi_common/bwi_dialog/src/models/' + str(name) + '.pkl', 'r') as f:
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

