__author__ = ['aishwarya', 'rodolfo']

import random
import sys

path = 'resources/parser_training/'

class ParserTrainer:
    def __init__(self):
        #Initializes data structure for generating phrases. 
        self.templates = []
        self.tags = {}
        self.names = []
        self.names_possessive = []
        self.adjectives = []
        self.nouns = []
        self.used_adjectives = False

        self.genTags()
        self.genTemplates()

        #These methods MUST be called AFTER genTags().  
        self.genNames()
        self.genAdjectives()
        self.genNouns()

        #Variables to hold data to be saved.
        self.phrase = None
        self.denotations = {} #Keeps specific word denotations.
        self.denotation = None # keeps denotation for entire phrase. 
        self.semantic_form = None
        self.semantic_data = {}

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
        #Templates are all held in lists. 
        self.templates.append(["walk", [line for line in open(path + 'walkTemplates.txt', 'r')]])
        self.templates.append(["search", [line for line in open(path + 'searchTemplates.txt', 'r')]])
        self.templates.append(["bring", [line for line in open(path + 'bringTemplates.txt', 'r')]])
        self.templates.append(["walk_possessive", [line for line in open(path + 'walkPossessiveTemplates.txt', 'r')]])

    #Generates a list of adjectives as well as the determiners that precede each one. 
    def genAdjectives(self):
        for line in self.tags["<A>"]:
            splitList = line.split(':')
            determiner = splitList[1].strip()
            adjective = splitList[0].strip()

            self.adjectives.append([determiner, adjective])

        #print "Adjectives: " + str(self.adjectives)

    def genNouns(self):
        for line in self.tags["<NO>"]:
            splitList = line.split(':')
            determiner = splitList[1].strip()
            noun = splitList[0].strip()

            self.nouns.append([determiner, noun])

        #print "Nouns: " + str(self.nouns)

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
        templateListIndex = random.randint(0, len(self.templates) -1) 
        templateType = self.templates[templateListIndex][0]
        templateList = self.templates[templateListIndex][1]

        #Picks a random template from that action's templates. 
        templateIndex = random.randint(0, len(templateList) - 1)
        template = templateList[templateIndex]

        command = []

        for word in template.split():
            command.append(self.processWord(word))

        self.genDenotation(templateType)
        self.genSemanticForm(templateType)

        print 'command = ', command
        return ' '.join(command)

    def genSemanticForm(self, templateType):
        if templateType == "walk_possessive":
            name = self.semantic_data["name"]

            self.semantic_form = "at(the(lambda x:e.(hasoffice("
            self.semantic_form += name + ", x))))"
        elif templateType == "bring":
            if self.used_adjectives:
                self.semantic_form = "bring(a(" 
                self.semantic_form += self.semantic_data["item form"] + "),"
                self.semantic_form += self.semantic_data["name"] + ")"
            else:
                #Cases equivalent.
                self.semantic_form = self.denotation
            


    #Generates denotation for phrase.
    #Also generates semantic form if case matches. 
    def genDenotation(self, templateType):
        if templateType == "walk":
            self.denotation = "at(" + self.denotations["<P>"] + ")"
            self.semantic_form = self.denotation
        elif templateType == "bring":
            self.denotation = "bring("
            self.denotation += self.denotations["<I>"] + ","
            self.denotation += self.denotations["<N>"] + ")"
            self.semantic_form = self.denotation

        elif templateType == "search":
            self.denotation = "searchroom("
            self.denotation += self.denotations["<N>"] + ","
            self.denotation += self.denotations["<P>"] + ")"
            self.semantic_form = self.denotation
        elif templateType == "walk_possessive":
            self.denotation = "at(" + self.denotations["<PP>"] + ")"

    def genAdjectiveDescribed(self):
        adjectives = list(self.tags["<A>"])

        nouns = self.tags["<NO>"]
        semantic_form = ""

        #0 - 2 adjectives will be randomly chosen. 
        numAdjectives = random.randint(0, 2)
            
        #Picks a random noun. 
        nounTuple = nouns[random.randint(0, len(nouns) - 1)].split(':')
        noun_determiner = nounTuple[1].strip()
        noun = nounTuple[0].strip()

        #Phrase to build. 
        phrase = ""

        #Generates unique adjectives for the phrase. 
        for i in range(0, numAdjectives):
            adjectiveIndex = random.randint(0, len(adjectives) - 1)
            adjectiveTuple = adjectives[adjectiveIndex].split(':')

            adjective = adjectiveTuple[0].strip()
            determiner = adjectiveTuple[1].strip()

            #Begins phrase. 
            if i == 0:
                phrase = determiner + " "
                    
            #Adds adjective to phrase with comma if necessary. 
            if i < numAdjectives - 1:
                phrase += adjective + ", "
            else:
                phrase += adjective + " "
                    
            #Adds adjective to semantic form. 
            semantic_form += "and(" + adjective + "(x), "

            del adjectives[adjectiveIndex]

        #If no adjective, then adds determiner. 
        if numAdjectives == 0:
            phrase += noun_determiner + " "

        #Adds noun to phrase and semantic form. 
        phrase += noun
        semantic_form += noun + "(x)"

        #Finishes preparing semantic form. 
        for i in range(0, numAdjectives):
            semantic_form += ")"

        semantic_form = "lambda x:e.(" + semantic_form + ")"

        #Stores denotation and preliminary semantic form. 
        self.denotations["<I>"] = noun
        self.semantic_data["item form"] = semantic_form
    
        #Used in genSemanticForm method for disambiguating the case. 
        self.used_adjectives = True

        return phrase

    def genNormalItem(self):
        expansionIndex = random.randint(0, len(self.tags["<I>"]) - 1)
        expansionList = self.tags["<I>"][expansionIndex].split(':')
        expansion = expansionList[1].strip()
        denotation = expansionList[0].strip()

        #Stores denotation. 
        self.denotations["<I>"] = denotation

        #Did not use adjectives, this comes up when generating the semantic form. 
        self.used_adjectives = False

        return expansion
    
    def processWord(self, word): 
        #Expands name from name list of names that was built. 
        if word == "<N>":
            nameList = self.names[random.randint(0, len(self.names) - 1)]
            name = nameList[0]
            denotation = nameList[1]

            #Stores denotation
            self.denotations[word] = denotation

            #Stores name to create semantic form later. 
            self.semantic_data["name"] = denotation

            return name
        elif word == "<I>":
            self.genNormalItem()
           
        elif word == "<PP>":
            nameList = self.names_possessive[random.randint(0, len(self.names_possessive) - 1)]
            name = nameList[0]
            denotation = nameList[1]
            office = nameList[2]

            #Stores denotation. 
            self.denotations[word] = office

            #Saves name for semantic form. 
            self.semantic_data["name"] = denotation
            
            #Returns name with possessive. 
            return name
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
    parser_trainer = ParserTrainer()
    print parser_trainer.tags.keys()
    x = raw_input()
    while True :
        print parser_trainer.genCommand()
        print parser_trainer.semantic_form
        x = raw_input()
        if x == 'n' :
            break
    
