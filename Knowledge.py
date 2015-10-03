__author__ = 'aishwarya'

# TODO: Automate filling as many of these as possible from knowledge base

class Knowledge:
    yes = 'yes'
    no = 'no'
    unk = '-UNK-'

    def __init__(self):
        self.goal_actions = ['searchroom', 'speak_t', 'speak_e', 'remind', 'askperson']
        self.goal_params = ['patient', 'location', 'recipient']

        # This is kept as a single vector common to all values so that hopefully 
        # some opearitons involving them can be made matrix operations and implemented
        # efficiently using numpy
        self.goal_params_values = [None, 'peter', 'ray', 'dana', 'kazunori', 'matteo', 'shiqi', 'jivko', 'stacy' 'yuqian', 'max', 'pato', 'bwi', 'bwi-meeting', '3516', '3508', '3512', '3510', '3402', '3418', '3420', '3432', '3502', '3414b']
        
        self.dialog_actions = ['repeat_goal', 'confirm_action', 'request_missing_param']

        self.goal_change_prob = 0.0
        
        self.param_relevance = dict()
        self.set_param_relevance()

    # This gives a 0-1 value for whether a param is relevant for an action. 
    # Assumes the following form of actions - 
    #   searchroom - patient (who/what to search for), location (where to search)
    #   speak_t - patient (what to say)
    #   speak_e - patient (what to say)
    #   remind - patient (event you are reminding someone of), location (where is the 
    #            event), recipient (the person you are reminding)
    #   askperson - patient (the person you ask), recipient (the person you ask about)
    def set_param_relevance(self) :
        self.param_relevance = dict()
        for action in self.goal_actions :
            self.param_relevance[action] = dict()
            for param in self.goal_params :
                self.param_relevance[action][param] = 0
        
        self.param_relevance['searchroom']['patient'] = 1
        self.param_relevance['searchroom']['location'] = 1     
        self.param_relevance['speak_t']['patient'] = 1
        self.param_relevance['speak_e']['patient'] = 1
        self.param_relevance['remind']['patient'] = 1
        self.param_relevance['remind']['location'] = 1
        self.param_relevance['remind']['recipient'] = 1
        self.param_relevance['askperson']['patient'] = 1
        self.param_relevance['askperson']['recipient'] = 1

        


    
