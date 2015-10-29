__author__ = 'aishwarya'

import sys, random, math, copy
from Knowledge import Knowledge
from HISBeliefState import HISBeliefState
from SummaryState import SummaryState
from PomdpGpSarsaPolicy import PomdpGpSarsaPolicy
from SystemAction import SystemAction
from Utterance import Utterance
from Action import Action

class PomdpDialogAgent :

    def __init__(self, parser, grounder, policy, u_input, output, parse_depth=10):
        self.parser = parser
        self.grounder = grounder
        self.parse_depth = parse_depth
        self.input = u_input
        self.output = output

        self.knowledge = Knowledge()    
        self.state = HISBeliefState(self.knowledge)  
        self.policy = PomdpGpSarsaPolicy(self.knowledge, True)
        self.previous_system_action = SystemAction('repeat_goal')  
        self.n_best_utterances = None

        # define action names and functions they correspond to
        self.dialog_actions = self.knowledge.system_dialog_actions
        self.dialog_action_functions = [self.request_user_initiative, self.confirm_action, self.request_missing_param]

        self.first_turn = True

    # Returns True if the dialog terminates on its own and False if the u
    # user entered stop
    def run_dialog(self) :
        self.state = HISBeliefState(self.knowledge)
        print 'Belief state: ', str(self.state) 
        summary_state = SummaryState(self.state)
        (dialog_action, dialog_action_args) = self.policy.get_initial_action(summary_state)    
        
        while True :
            self.state.increment_dialog_turns()
            if dialog_action == 'take_action' :
                # Submit the action given by the policy
                action = dialog_action_args[0]
                self.output.say("Action: " + str(action))
                self.output.say("Was this the correct action? (y/n) : ")
                response = self.input.get()    
                if response.lower() == 'y' or response.lower() == 'yes' :
                    self.policy.update_final_reward(self.knowledge.correct_action_reward)
                else :
                    self.policy.update_final_reward(self.knowledge.wrong_action_reward)
                return True
            else :
                previous_system_action = dialog_action_args[0]
                # Take the dialog action
                response = self.dialog_action_functions[self.dialog_actions.index(dialog_action)](dialog_action_args)
                if response == 'stop' :
                    self.policy.update_final_reward(self.knowledge.wrong_action_reward)
                    return False
                # Belief monitoring - update belief based on the response
                self.update_state(response)
                print 'Belief state: ', str(self.state) 
                
                reward = self.knowledge.per_turn_reward
                summary_state = SummaryState(self.state)
                # Get the next action from the policy
                (dialog_action, dialog_action_args) = self.policy.get_next_action(reward, summary_state)
            self.first_turn = False
        
    # Request the user to state/repeat their goal
    def request_user_initiative(self, args):
        if self.first_turn :
            self.output.say("How can I help?")
        else :
            self.output.say("Can you restate your question or command?")
        response = self.input.get()
        return response

    # Request a missing action parameter
    def request_missing_param(self, args):
        theme = args[1]
        self.output.say("What is the " + theme + " of the action you'd like me to take?")
        response = self.input.get()
        return response

    # Parse the user response and update belief state
    def update_state(self, response) :
        # get n best parses for confirmation
        n_best_parses = self.parser.parse_expression(response, n=self.parse_depth)

        # Create an n-best list of Utterance objects along with probabiltiies 
        # obtained fromt ehir confidences
        self.get_n_best_utterances_from_parses(n_best_parses)
        # Update the belief state
        print '\nGoing to update belief state'
        print 'Previous system action - ', str(self.previous_system_action)
        print 'Utterances - ', '\n'.join([str(utterance) for utterance in self.n_best_utterances])
        self.state.update(self.previous_system_action, self.n_best_utterances)

    # Confirm a (possibly partial) action
    def confirm_action(self, args):
        # with reverse parsing, want to confirm(a.name(a.params))
        # for now, just use a.name and a.params raw
        system_action = args[0]
        params = []
        goal = system_action.referring_goal
        if system_action.referring_params is not None :
            param_order = self.knowledge.param_order[goal]
            for param_name in param_order :
                if param_name in system_action.referring_params :
                    params.append[system_action.referring_params[param_name]]
        self.output.say("I should take action " + goal +" involving " +
                        ','.join([str(p) for p in params if p is not None]) + "?")
        response = self.input.get()
        return response

    # Converts an object of type Action to type Utterance assuming it is
    # informing the system of goal and param details
    def convert_action_to_utterance(self, action) :
        utterance = None
        if action is not None :
            goal = action.name
            if action.params is None :
                utterance = Utterance('inform', goal)
            else :
                params = dict()
                for (idx, param_val) in enumerate(action.params) :
                    param_name = self.knowledge.param_order[goal][idx]
                    if param_val != 'UNK_E' :
                        params[param_name] = param_val
                utterance = Utterance('inform', goal, params)
        return utterance

    def create_utterances_of_parse(self, parse) :
        # First check if it is a confirmation or denial
        if parse.idx == self.parser.ontology.preds.index('yes'):
            #print 'Is affirm'
            return [Utterance('affirm')]
        elif parse.idx == self.parser.ontology.preds.index('no'):
            #print 'Is deny'
            return [Utterance('deny')]
        
        # Now check if it a full inform - mentions goal and params
        try:
            #print 'Trying to get an action'
            p_action = self.get_action_from_parse(parse)
        except SystemError:
            p_action = None

        if p_action is not None :
            #print 'Got an action. Will convert and return'
            return [self.convert_action_to_utterance(p_action)]

        # if this failed, try again allowing missing lambdas to become UNK without token ties
        # print "Going to retry parsing"
        UNK_root = copy.deepcopy(parse)
        curr = UNK_root
        heading_lambdas = []
        if curr.is_lambda and curr.is_lambda_instantiation:
            heading_lambdas.append(curr.lambda_name)
            curr = curr.children[0]
        if curr.children is not None:
            for i in range(0, len(curr.children)):
                if curr.children[i].is_lambda and curr.children[i].lambda_name in heading_lambdas:
                    curr.children[i] = self.parser.create_unk_node()
        try:
            #print 'Trying to get UNK action'
            p_unk_action = self.get_action_from_parse(UNK_root)
        except SystemError:
            p_unk_action = None
            
        if p_unk_action is not None :
            #print 'Got UNK action. Going to convert and return'
            return [self.convert_action_to_utterance(p_unk_action)]
            
        # Getting a complete action failed so assume this is a param  
        #print 'Trying to ground as param value'
        #print "\nGrounding ", self.parser.print_parse(parse)
        try :
            g = self.grounder.groundSemanticNode(parse, [], [], [])
            answers = self.grounder.grounding_to_answer_set(g)
            utterances = []
            if type(answers) == 'str' :
                answers = [answers]
            for answer in answers :
                goal = self.previous_system_action.referring_goal
                params = dict()
                param_name = 'patient' # TODO: Something more intelligent than
                                       # defaulting when we don't know the goal
                if goal is not None :
                    param_name = self.knowledge.param_order[goal][idx] 
                params[param_name] = answer
                utterance = Utterance('inform', goal, params)      
                utterances.append(utterance)
            return utterances
        except TypeError as e :
            print e.message
            
    def get_n_best_utterances_from_parses(self, n_best_parses) :
        print "In get_n_best_utterances_from_parses"
        print "No of parses = ", len(n_best_parses)
        sum_exp_conf = 0.0
        self.n_best_utterances = []
        for (parse, parse_list, conf) in n_best_parses :
            #print "\nParsing ", self.parser.print_parse(parse)
            #print 'conf = ', conf
            utterances = self.create_utterances_of_parse(parse)
            #print "Returned"
            if utterances is not None and len(utterances) > 0:
                #print "Got ", len(utterances), " utterances"
                for utterance in utterances :
                    #print str(utterance)
                    utterance.parse_prob = math.exp(conf) / len(utterances)
                    self.n_best_utterances.append(utterance)
                    sum_exp_conf += utterance.parse_prob
        sum_exp_conf += self.knowledge.obs_by_non_n_best_prob
        for utterance in self.n_best_utterances :
            utterance.parse_prob /= sum_exp_conf
            
        print "N-best utterances: "
        for utterance in self.n_best_utterances :
            print str(utterance)

    def read_in_utterance_action_pairs(self, fname):
        f = open(fname,'r')
        f_lines = f.readlines()
        pairs = []
        for i in range(0,len(f_lines),3):
            t = self.parser.tokenize(f_lines[i].strip())
            a_str = f_lines[i+1].strip()
            a_name, a_params_str = a_str.strip(')').split('(')
            a_params = a_params_str.split(',')
            for j in range(0, len(a_params)):
                if a_params[j] == "True":
                    a_params[j] = True
                elif a_params[j] == "False":
                    a_params[j] = False
            pairs.append([t, Action(a_name, a_params)])
        f.close()
        return pairs

    def train_parser_from_utterance_action_pairs(self, pairs, epochs=10, parse_beam=10):
        for e in range(0, epochs):
            print "training epoch "+str(e)
            random.shuffle(pairs)
            train_data = []
            num_correct = 0
            for t, a in pairs:
                n_best_parses = self.parser.parse_tokens(t, n=parse_beam)
                if len(n_best_parses) == 0:
                    print "WARNING: no parses found for tokens "+str(t)
                    continue
                a_chosen = None
                a_candidate = None
                correct_found = False
                for i in range(0, len(n_best_parses)):
                    try:
                        # print "parse: " + self.parser.print_parse(n_best_parses[i][0])  # DEBUG
                        # print self.parser.print_semantic_parse_result(n_best_parses[i][1])  # DEBUG
                        a_candidate = self.get_action_from_parse(n_best_parses[i][0])
                        # print "candidate: "+str(a_candidate)  # DEBUG
                    except SystemError:
                        a_candidate = Action()
                    if i == 0:
                        a_chosen = a_candidate
                    if a_candidate.__eq__(a):
                        correct_found = True
                        self.parser.add_genlex_entries_to_lexicon_from_partial(n_best_parses[i][1])
                        break  # the action matches gold and this parse should be used in training
                if not correct_found:
                    print "WARNING: could not find correct action '"+str(a)+"' for tokens "+str(t)
                if a_chosen != a_candidate:
                    train_data.append([t, n_best_parses[0], a_chosen, n_best_parses[i], a])
                else:
                    num_correct += 1
            print "\t"+str(num_correct)+"/"+str(len(pairs))+" top choices"
            if num_correct == len(pairs):
                print "WARNING: training converged at epoch "+str(e)+"/"+str(epochs)
                return True
            self.parser.learner.learn_from_actions(train_data)
        return False

    def get_action_from_parse(self, root):
        # print "parse to get action from: " + self.parser.print_parse(root)  # DEBUG
        root.set_return_type(self.parser.ontology)  # in case this was not calculated during parsing

        # execute action from imperative utterance
        if root.return_type == self.parser.ontology.types.index('a'):
            # assume for now that logical connectives do not operate over actions (eg. no (do action a and action b))
            # ground action arguments
            action = self.parser.ontology.preds[root.idx]
            # print "action: "+action  # DEBUG
            g_args = []
            for arg in root.children:
                g = self.grounder.groundSemanticNode(arg, [], [], [])
                # print "arg: "+str(g)  # DEBUG
                answer = self.grounder.grounding_to_answer_set(g)
                if len(answer) == 0:
                    if action == "speak_t":
                        g_args.append(False)
                    elif action == "speak_e":
                        g_args.append(None)
                    else:
                        raise SystemError("action argument unrecognized. arg: '"+str(answer)+"'")
                elif len(answer) > 1:
                    raise SystemError("multiple interpretations of action argument renders command ambiguous. arg: '"+str(answer)+"'")
                else:
                    if action == "speak_t":
                        g_args.append(True)
                    else:
                        g_args.append(answer[0])
            # print "args: "+str(g_args)  # DEBUG
            return Action(action, g_args)

        else:
            raise SystemError("cannot get action from return type "+str(self.parser.ontology.types[root.return_type]))
