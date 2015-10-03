__author__ = 'aishwarya'

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
