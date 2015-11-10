__author__ = 'aishwarya'

import sys, itertools

def init_values_for_pomdp_gp_sarsa() :
    probs = xrange(0, 10) * 0.1
    num_goals = xrange(1, 6)
    num_uncertain_params = xrange(0, 4) + [sys.maxint]
    num_dialog_turns = xrange(0, 10)
    yes_no = ['yes', 'no']
    utterance_type = [None, 'inform', 'affirm', 'deny']
    values = list(itertools.product(*[probs, probs, num_goals, num_uncertain_params, num_dialog_turns, yes_no, utterance_type]))
    for (top_prob, sec_prob, num_goals, num_uncertain_params, num_dialog_turns, match, utterance_type) in values :
        if num_goals == 1 :
            if num_uncertain_params == 0 :
                if top_prob < 0.3 :
                    return 'request_missing_param'
                elif top_prob < 0.9 :
                    return 'confirm_action'
                else :
                    return 'repeat_goal'
            else :
                if num_uncertain_params > 0 :
                    return 'request_missing_param'
        else :
            return 'repeat_goal'
    
