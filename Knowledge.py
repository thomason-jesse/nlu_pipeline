__author__ = 'aishwarya'

# TODO: Automate filling as many of these as possible from knowledge base

goal_actions = ['searchroom', 'speak_t', 'speak_e', 'remind', 'askperson']
goal_params = ['patient', 'recipient', 'location']

# This is kept as a single vector common to all values so that hopefully 
# some opearitons involving them can be made matrix operations and implemented
# efficiently using numpy
goal_params_values = ['-NONE-', 'peter', 'ray', 'dana', 'kazunori', 'matteo', 'shiqi', 'jivko', 'stacy' 'yuqian', 'max', 'pato', 'bwi', 'bwi meeting', '3516', '3508', '3512', '3510', '3402', '3418', '3420', '3432', '3502', '3414b']

dialog_actions = ['repeat_goal', 'confirm_action', 'request_missing_param']



