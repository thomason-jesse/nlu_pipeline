import ctypes
import threading
import pygame
import random
import sys

class ScriptGenerator:
    def __init__(self, displayInfo, user_id, mode = "normal", recording = True):
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

        #Mode for generating items. 
        self.mode = mode

        #Variables to hold data to be saved.
        self.user_id = user_id
        self.phrase_num = 0
        self.phrase = None
        self.denotations = {} #Keeps specific word denotations.
        self.denotation = None # keeps denotation for entire phrase. 
        self.semantic_form = None
        self.semantic_data = {}

        #Sets font for displaying text. 
        self.font = pygame.font.SysFont('monospace', 15)
        self.textColor = (255, 255, 255)
        self.background = (191, 87, 0)
        self.screen_w = displayInfo.current_w
        self.screen_h = displayInfo.current_h

    def genPhrase(self, screen):
        #Generates the phrase itself. (i.e. the text)
        self.phrase = self.genCommand()    

        #The text object to render to screen. 
        self.phraseText = self.font.render(self.phrase, 1, self.textColor)

        #Updates display. 
        screen.fill(self.background)
        screen.blit(self.phraseText, self.calcPos())
        pygame.display.update()

    #Calculates the position of the text on the screen. 
    def calcPos(self):
        width = self.phraseText.get_width()
        height = self.phraseText.get_height()

        diff_w = self.screen_w - width
        diff_h = self.screen_h - height

        if diff_w <= 0:
            x = 0
        else:
            x = diff_w / 2

        if diff_h <= 0:
            y = 0
        else:
            y = diff_h / 2

        return (x,y)

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
	self.templates.append(["walk", [line for line in open('walkTemplates.txt', 'r')]])
	self.templates.append(["search", [line for line in open('searchTemplates.txt', 'r')]])
        self.templates.append(["bring", [line for line in open('bringTemplates.txt', 'r')]])
	self.templates.append(["walk_possessive", [line for line in open('walkPossessiveTemplates.txt', 'r')]])

    #Generates a list of adjectives as well as the determiners that precede each one. 
    def genAdjectives(self):
        for line in self.tags["<A>"]:
            splitList = line.split(':')
            determiner = splitList[0].strip()
            adjective = splitList[1].strip()

            self.adjectives.append([determiner, adjective])

    def genNouns(self):
        for line in self.tags["<NO>"]:
            splitList = line.split(':')
            determiner = splitList[0].strip()
            noun = splitList[1].strip()

            self.nouns.append([determiner, noun])

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

        return ' '.join(command)

    def genSemanticForm(self, templateType):
        if templateType == "walk_possessive":
            name = self.semantic_data["name"]

            self.semantic_form = "walk(the(lambda x:e.(and(office(x), possesses(x, "
            self.semantic_form += name + ")))"
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
            self.denotation = "walk(" + self.denotations["<P>"] + ")"
            self.semantic_form = self.denotation
        elif templateType == "bring":
            self.denotation = "bring("
            self.denotation += self.denotations["<I>"] + ","
            self.denotation += self.denotations["<N>"] + ")"
            
            #If normal bring action, then denotation == semantic_form. 
            if self.mode == "normal":
                self.semantic_form = self.denotation

        elif templateType == "search":
            self.denotation = "searchroom("
            self.denotation += self.denotations["<N>"] + ","
            self.denotation += self.denotations["<P>"] + ")"
            self.semantic_form = self.denotation
        elif templateType == "walk_possessive":
            self.denotation = "walk(" + self.denotations["<PP>"] + ")"

    def genAdjectiveDescribed(self):
        adjectives = list(self.tags["<A>"])

        nouns = self.tags["<NO>"]
        semantic_form = ""

        #0 - 2 adjectives will be randomly chosen. 
        numAdjectives = random.randint(0, 2)
            
        #Picks a random noun. 
        nounTuple = nouns[random.randint(0, len(nouns) - 1)].split(':')
        noun_determiner = nounTuple[0].strip()
        noun = nounTuple[1].strip()

        #Phrase to build. 
        phrase = ""

        #Generates unique adjectives for the phrase. 
        for i in range(0, numAdjectives):
            adjectiveIndex = random.randint(0, len(adjectives) - 1)
            adjectiveTuple = adjectives[adjectiveIndex].split(':')

            adjective = adjectiveTuple[1].strip()
            determiner = adjectiveTuple[0].strip()

            #Begins phrase. 
            if i == 0:
                determiner = adjectives[adjectiveIndex][0]
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
        expansion = expansionList[0].strip()
        denotation = expansionList[1].strip()

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
            if self.mode == "normal":
                return self.genNormalItem()
            elif self.mode == "adjective":
                return self.genAdjectiveDescribed()
            elif self.mode == "mix":
                #Picks either an adjective described or a regular item. 
                if random.randint(0, 1):
                    return self.genAdjectiveDescribed()
                else:
                    return self.genNormalItem() 
            else:
                print "Mode not correctly set!"
                sys.exit()
           
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

    def saveData(self):
        phraseFile = open('results/phrases/' + str(self.user_id) + '_' + 'phrase' + '_' + str(self.phrase_num), 'w')
        denotationFile = open('results/denotations/' + str(self.user_id) + '_' + 'denotation' + '_' + str(self.phrase_num), 'w')
        semanticFile = open('results/semantic_forms/' + str(self.user_id) + '_' + 'semantic' + '_' + str(self.phrase_num), 'w')

        #Writes data. 
        phraseFile.write(self.phrase)
        denotationFile.write(self.denotation)
        semanticFile.write(self.semantic_form)

        print "----------------------------------"
        print "PHRASE: " + self.phrase
        print "DENOTATION: " + self.denotation
        print "SEMANTIC FORM: " + self.semantic_form
        print "----------------------------------"

        #Closes file. 
        phraseFile.close()
        denotationFile.close()
        semanticFile.close()

        #Updates phrase number. 
        self.phrase_num += 1

class Recorder:
    def __init__(self, user_id, mode):
        import ctypes

        if True: #TODO Allow path specification is None:
            self.libHandle = ctypes.CDLL("./record.so")
        else:
            self.libHandle = ctypes.CDLL(path)

        self.user_id = user_id
        self.mode = mode
        self.phrase_num = 0
        self.scriptGenerator = None

        #Initialize microphone. 
        self.libHandle.initMics()

    def __del__(self):
        self.libHandle.closeMics()

    def record(self):
        while not self.libHandle.isInterrupted():
            self.libHandle.record1600Hz("results/recordings/" + self.user_id + "_recording_" + str(self.phrase_num))
            self.phrase_num += 1

    def recordToggle(self):
        #Initializes pygame window to read spacebar presses. 
        pygame.init()

        #Makes mouse invisible. 
        pygame.mouse.set_visible(False)

        #Gets monitor information and uses it to go to fullscreen mode. 
        displayInfo = pygame.display.Info()
        screen = pygame.display.set_mode((displayInfo.current_w -100, displayInfo.current_h-100))

        #Information for "recording" circle. 
        red = (255, 0, 0)
        orange = (191, 87, 0)
        circPos = (displayInfo.current_w - 200, 200)

        print "\nPress [SPACEBAR] to record"

        running = True

        #Creates the phrase generating object. 
        self.scriptGenerator = ScriptGenerator(displayInfo, self.user_id, self.mode)
        self.scriptGenerator.genPhrase(screen)

        while running:
            pygame.event.pump()
            keys = pygame.key.get_pressed()

            if (keys[pygame.K_RCTRL] or keys[pygame.K_LCTRL]) and keys[pygame.K_c]:
                running = False

                self.libHandle.interruptRecord()

            if keys[pygame.K_SPACE]:
                #Starts recording from mic to file. 
                self.libHandle.startRecord()

                #Loops while user is holding space bar. 
                while keys[pygame.K_SPACE]:
                    pygame.event.pump()
                    keys = pygame.key.get_pressed()

                #Draws circle to signify recording. 
                pygame.draw.circle(screen, red, circPos, 20, 0) 
                pygame.display.update()

                recording = True

                #Waits for user to press spacebar again to stop recording. 
                while recording:
                    pygame.event.pump()
                    keys = pygame.key.get_pressed()

                    if keys[pygame.K_SPACE]:
                        recording = False
                        screen.fill(orange)
                        pygame.display.update()

                    #Sleeps to free up cpu. 
                    pygame.time.wait(100)

                #Stops recording when they release it. 
                self.libHandle.stopRecord()
                recording = False

                #Saves phrase, denotation, and semantic form to file. 
                self.scriptGenerator.saveData()

                #Waits for user to get next phrase.     
                while not keys[pygame.K_RIGHT]:
                    pygame.event.pump()
                    keys = pygame.key.get_pressed()

                #Waits for user to let go of key.     
                while keys[pygame.K_RIGHT]:
                    pygame.event.pump()
                    keys = pygame.key.get_pressed()

                self.scriptGenerator.genPhrase(screen)

            #Sleeps to free up cpu.
            pygame.time.wait(100)

            
        pygame.quit()

    def recordUser(self):

        #Creates threads for recording. 
        thread1 = threading.Thread(target = self.record)
        thread2 = threading.Thread(target = self.recordToggle)

        #Starts thread execution. 
        thread1.start()
        thread2.start()

        #Waits for recording threads to finish. 
        thread1.join()
        thread2.join()

#Ensures that arguments were passed correctly.
if not len(sys.argv) >= 2:
    print "Usage: python record.py [user_id] [mode] [record]"
    sys.exit()

user_id = sys.argv[1]

if len(sys.argv) >= 3:
    mode = sys.argv[2]

    if not (mode == "normal" or mode == "adjective" or mode == "mix"):
        print "Mode not recognized, correct values: normal, adjective, mix."
        sys.exit()
else: 
    mode = "normal"

if len(sys.argv) == 4:
    record = sys.argv[3]
    
    if not (record == "y" or record == "n"):
        print "record value must be \'y\' or \'n\'."
else:
    record = True

#Sets up recording interface if necessary. 
if record:
    recorder = Recorder(user_id, mode)
    recorder.recordUser()
else:
    pass
    #TODO script generator without recording environment. 
