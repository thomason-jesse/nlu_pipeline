#!/usr/bin/env python
__author__ = ['aishwarya', 'rodolfo']

import random
import sys
import Ontology
import Lexicon
import CKYParser
from utils import *

path = '/u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/resources/parser_training/'

class ParserTrainer:
    def __init__(self):
        #Initializes data structure for generating phrases. 
        self.templates = dict()
        self.tags = {}
        self.names = []
        self.names_possessive = []

        self.genTags()
        self.genTemplates()

        #These methods MUST be called AFTER genTags().  
        self.genNames()

        #Variables to hold data to be saved.
        self.phrase = None
        self.denotations = dict() #Keeps specific word denotations.
        self.denotation = None # keeps denotation for entire phrase. 
        self.semantic_form = None
        self.semantic_forms = dict()
        self.people_used = list()
        
        #print 'names possessive ', self.names_possessive   # DEBUG

    #Generates dictionary of tags and their elements from lex.txt. 
    def genTags(self):
        self.tags = {}

        lexFile = open(path + 'lex.txt', 'r')
        line = lexFile.readline().rstrip()

        while not line == "":
            #<C> marks the start of a category (i.e. a list for the tag). 
            if line == "<C>":
                #The tag should be right underneath <C> in lex.txt. 
                tag = lexFile.readline().rstrip()
                self.tags[tag] = []

                line = lexFile.readline().rstrip()

                #This marks the end of the category. 
                while not line == "</C>":
                    self.tags[tag].append(line)

                    line = lexFile.readline().rstrip()

            line = lexFile.readline().rstrip()

        lexFile.close()

    #Generates templates from files. 
    def genTemplates(self):
        self.templates['walk'] = [line for line in open(path + 'walk_easy.txt', 'r')]
        self.templates['search'] = [line for line in open(path + 'search_easy.txt', 'r')]
        self.templates['bring'] = [line for line in open(path + 'bring_easy.txt', 'r')]
        
    #Generates list of all possible references to people. 
    def genNames(self): 
        #Adds positions by themselves. 
        for position in self.tags["<PS>"]:
            splitList = position.split(':')
            position = splitList[0].strip()
            denotation = splitList[1].strip()

            self.names.append([position, denotation])

        for fullName in self.tags["<FN>"]:
            splitList = fullName.split(':')
            name = splitList[0].strip().split('-')
            titles = splitList[1].strip().split('_')
            denotation = splitList[2].strip()
            office = splitList[3].strip()

            #Names in person's full name.
            first_name = name[0]
            nickname = name[1]
            last_name = name[2]

            #Adds first name by itself. 
            if not first_name == "NA":
                self.names.append([first_name, denotation])
                
                if not office == "NA":
                    self.names_possessive.append([first_name + "'s", denotation, office])

            #Adds nickname by itself. 
            if not nickname == "NA":
                self.names.append([nickname, denotation])

                if not office == "NA":
                    self.names_possessive.append([nickname + "'s", denotation, office])

            #Adds title, first, last
            for title in titles:
                if not last_name == "NA":
                    nameString = title + " " + first_name + " " + last_name
                else:
                    nameString = title + " " + first_name

                self.names.append([nameString, denotation])
                
                if not office == "NA":
                    self.names_possessive.append([nameString + "'s", denotation, office])

            #Adds first, last. 
            if not last_name == "NA":
                nameString = first_name + " " + last_name
                self.names.append([nameString, denotation])
                
                if not office == "NA":
                    self.names_possessive.append([nameString + "'s", denotation, office]) 

            #Adds nickname, last.
            if not nickname == "NA" and not last_name == "NA":
                nameString = nickname + " " + last_name
                self.names.append([nameString, denotation])
                
                if not office == "NA":
                    self.names_possessive.append([nameString + "'s", denotation, office])

            #Adds title, last. 
            if not last_name == "NA":
                for title in titles:
                    nameString = title + " " + last_name
                    self.names.append([nameString, denotation])
                    
                    if not office == "NA":
                        self.names_possessive.append([nameString + "'s", denotation, office])
    

    def genCommand(self):
        #Picks a random action to take. 
        actions = self.templates.keys()
        actionIndex = random.randint(0, len(actions) -1) 
        templateType = actions[actionIndex]
        templateList = self.templates[templateType]

        #Picks a random template from that action's templates. 
        templateIndex = random.randint(0, len(templateList) - 1)
        template = templateList[templateIndex]

        command = []
        self.people_used = list()

        for word in template.split():
            command.append(self.processWord(word))
        
        if len(self.people_used) > 2 :
            print 'len(self.people_used) > 2 : check your code'
            sys.exit(1)
        elif len(self.people_used) == 2 :
            if self.people_used[0] == self.people_used[1] :
                # Search has same searchee and owner
                # Need a new command 
                return self.genCommand()

        self.genDenotationAndSemForm(templateType)

        #print 'command = ', command    # DEBUG
        return ' '.join(command)

    #Generates denotation for phrase.
    #Also generates semantic form if case matches. 
    def genDenotationAndSemForm(self, templateType):
        if templateType == "walk":
            self.denotation = "at(" + self.denotations["<P>"] + ")"
            self.semantic_form = "at(" + self.semantic_forms["<P>"] + ")"
        elif templateType == "bring":
            self.denotation = "bring("
            self.denotation += self.denotations["<I>"] + ","
            self.denotation += self.denotations["<N>"] + ")"
            self.semantic_form = self.denotation
        elif templateType == "search":
            self.denotation = "searchroom("
            self.denotation += self.denotations["<N>"] + ","
            self.denotation += self.denotations["<P>"] + ")"
            self.semantic_form = "searchroom("
            self.semantic_form += self.semantic_forms["<N>"] + ","
            self.semantic_form += self.semantic_forms["<P>"] + ")"
        
    def genNormalItem(self):
        expansionIndex = random.randint(0, len(self.tags["<I>"]) - 1)
        expansionList = self.tags["<I>"][expansionIndex].split(':')
        expansion = expansionList[1].strip()
        denotation = expansionList[0].strip()
        #print 'expansion = ', expansion, ', denotation = ', denotation # DEBUG

        #Stores denotation. 
        self.denotations["<I>"] = denotation
        self.semantic_forms["<I>"] = denotation
        return expansion
    
    def processWord(self, word): 
        #Expands name from name list of names that was built. 
        if word == "<N>":
            nameList = self.names[random.randint(0, len(self.names) - 1)]
            name = nameList[0]
            denotation = nameList[1]

            #Stores denotation
            self.denotations[word] = denotation
            self.semantic_forms[word] = denotation
            self.people_used.append(denotation)
            return name
        elif word == "<I>":
            return self.genNormalItem()
        elif word == '<P>' :
            r = random.random()
            if r < 0.5 :
                # Pick a direct room number
                expansionIndex = random.randint(0, len(self.tags[word]) - 1)
                expansionList = self.tags[word][expansionIndex].split(':')
                expansion = expansionList[0].strip()
                denotation = expansionList[1].strip()

                #Stores denotation. 
                self.denotations[word] = denotation
                self.semantic_forms[word] = denotation
                
                return expansion
            else :
                # Create a possessive form
                nameList = self.names_possessive[random.randint(0, len(self.names_possessive) - 1)]
                name = nameList[0]
                denotation = nameList[1]
                office = nameList[2]

                #Stores denotation. 
                self.denotations[word] = office
                self.semantic_forms[word] = 'the(lambda x:e.(hasoffice(' + denotation + ', x)))'
                self.people_used.append(denotation)
                
                #Returns name with possessive. 
                return name + ' office'
        #The rest of the cases can be replaced by a simple sampling from the tag's list. 
        elif word in self.tags:
            expansionIndex = random.randint(0, len(self.tags[word]) - 1)
            expansionList = self.tags[word][expansionIndex].split(':')
            expansion = expansionList[0].strip()
            denotation = expansionList[1].strip()

            #Stores denotation. 
            self.denotations[word] = denotation

            return expansion
        else:
            return word

    
if __name__ == '__main__' :
    ont = Ontology.Ontology(sys.argv[1])
    lex = Lexicon.Lexicon(ont, sys.argv[2])
    
    parser = CKYParser.CKYParser(ont, lex, use_language_model=False)
    
    ## Set parser hyperparams to best known values for training
    parser.max_multiword_expression = 3  # max span of a multi-word expression to be considered during tokenization
    parser.max_new_senses_per_utterance = 0  # max number of new word senses that can be induced on a training example
    parser.max_cky_trees_per_token_sequence_beam = 10  # for tokenization of an utterance, max cky trees considered
    parser.max_hypothesis_categories_for_unknown_token_beam = 2  # for unknown token, max syntax categories tried
    parser.max_expansions_per_non_terminal = 10

    # Create train set
    D = parser.read_in_paired_utterance_semantics(sys.argv[3])
    #for i in range(1, 10) :
        #command = parser_trainer.genCommand()
        ##print 'command = ', command, '\nsem form = ', parser_trainer.semantic_form
        #print command
        #print 'M : ' + parser_trainer.semantic_form
        #print 
        
        #ccg = parser.lexicon.read_category_from_str('M')
        #form = parser.lexicon.read_semantic_form_from_str(parser_trainer.semantic_form, 
                                                    #None, None, [], allow_expanding_ont=False)
        #form.category = ccg

        #D.append((command, form))
    converged = parser.train_learner_on_semantic_forms(D, 20, reranker_beam=1)
    print 'converged = ', converged
    if len(sys.argv) > 4 :
        filename = str(sys.argv[4])
    else :
        filename = '/u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/models/parser_condor_1000_easy.pkl'
    save_obj_general(parser, filename)
    
