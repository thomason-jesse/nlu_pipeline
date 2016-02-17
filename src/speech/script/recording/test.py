import random

class Generator():
    def __init__(self):
        self.templates = []
        self.tags = {}
        self.names = []

        self.genTags()
        self.genTemplates()
        self.genNames()

    #Generates dictionary of tags and their elements from lex.txt. 
    def genTags(self):
        self.tags = {}

        lexFile = open('lex.txt', 'r')
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
        self.templates.append([line for line in open('walkTemplates.txt', 'r')])
        self.templates.append([line for line in open('searchTemplates.txt', 'r')])
        self.templates.append([line for line in open('bringTemplates.txt', 'r')])

    #Generates list of all possible references to people. 
    def genNames(self):
        #Adds positions by themselves. 
        for position in self.tags["<PS>"]:
            self.names.append(position)

        #Adds nicknames. 
        for nickname in self.tags["<NN>"]:
            self.names.append(nickname)

        #Adds first names. 
        for f_name in self.tags["<FN>"]:
            self.names.append(f_name)

        #Adds title, first, last. 
        for title in self.tags["<T>"]:
            for f_name in self.tags["<FN>"]:
                for l_name in self.tags["<LN>"]:
                    self.names.append(title + " " + f_name + " " + l_name)

        #Adds first, last.
        for f_name in self.tags["<FN>"]:
            for l_name in self.tags["<LN>"]:
                self.names.append(f_name + " " + l_name)

        #Adds nickname, last.
        for nickname in self.tags["<NN>"]:
            for l_name in self.tags["<LN>"]:
                self.names.append(nickname + " " + l_name)

        #Adds title, last. 
        for title in self.tags["<T>"]:
            for l_name in self.tags["<LN>"]:
                self.names.append(title + " " + l_name)

    def genCommand(self):
        #Picks a random action to take. 
        templateType = self.templates[random.randint(0, len(self.templates) -1)]
        
        #Picks a random template from that action's templates. 
        templateIndex = random.randint(0, len(templateType))
        template = templateType[templateIndex]

        command = []

        for word in template.split():
            command.append(self.processWord(word))

        return ' '.join(command)

    def processWord(self, word):
        if word in self.tags:
            expansionIndex = random.randint(0, len(self.tags[word]) - 1)

            return self.tags[word][expansionIndex]
        #Name can be expanded using first, last, nickname, title, and position. 
        elif word == "<N>":
            return self.names[random.randint(0, len(self.names) - 1)]
        else:
            return word


generator = Generator()

print generator.genCommand()
