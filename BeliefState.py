__author__ = 'aishwarya'

from Knowledge import *

class BeliefState:

    def __init__(self):

        self.num_user_turns = 0
		self.num_goal_actions = len(goal_actions)
		self.num_goal_params = len(goal_params)
		self.num_goal_params_values = len(goal_params_values)
		
		# Initialize all belief variables to uniform distributions
		
		# g_a - Belief of the goal of (final action desired by) user
		self.goal_action_belief = [1.0/self.num_goal_actions] * self.num_goal_actions
		
		# g_p_i - Belief of the value of each param. For now this is a list of lists 
		# 		  but hopefully can be made a matrix later for efficient computation
		self.goal_params_belief = [[1.0/self.num_goal_params_values for x in range(self.num_goal_params_values)] for x in range(num_goal_params)] 
		
		# u_a - Belief of things clarified in previous user utterance
		self.utterance_action_belief = [1.0/self.num_goal_actions] * self.num_goal_actions
		
		# u_p_i - Belief of the value of each param clarified in previous user utterance. 
		# 		  For now this is a list of lists but hopefully can be made a matrix 
		#		  later for efficient computation
		self.utterance_params_belief = [[1.0/self.num_goal_params_values for x in range(self.num_goal_params_values)] for x in range(num_goal_params)] 
