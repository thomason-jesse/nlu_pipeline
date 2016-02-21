import ctypes
import threading
import pygame
import random
import sys

class ScriptGenerator:
    def __init__(self, displayInfo, user_id):
        #Initializes data structure for generating phrases. 
        self.templates = []
        self.tags = {}
        self.names = []

        self.genTags()
        self.genTemplates()
        self.genNames()

        #Variables to hold data to be saved.
        self.user_id = user_id
        self.phrase_num = 0
        self.phrase = None
        self.denotations = {} #Keeps specific word denotations.
        self.denotation = None # keeps denotation for entire phrase. 
        self.semantic_form = None

        #Sets font for displaying text. 
        self.font = pygame.font.SysFont('monospace', 15)
        self.textColor = (255, 255, 255)
        self.background = (191, 87, 0)
        self.screen_w = displayInfo.current_w
        self.screen_h = displayInfo.current_h

    def genPhrase(self, screen):
        self.phrase = self.genCommand()    

        self.phraseText = self.font.render(self.phrase, 1, self.textColor)

        screen.fill(self.background)
        screen.blit(self.phraseText, self.calcPos())
        pygame.display.update()

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

            #Adds first name by itself. 
            if not name[0] == "NA":
                self.names.append([name[0], denotation])

            #Adds nickname by itself. 
            if not name[1] == "NA":
                self.names.append([name[1], denotation])

            #Adds title, first, last
            for title in titles:
                if not name[2] == "NA":
                    nameString = title + " " + name[0] + " " + name[2]
                else:
                    nameString = title + " " + name[0]

                self.names.append([nameString, denotation])

            #Adds first, last. 
            if not name[2] == "NA":
                self.names.append([name[0] + " " + name[2], denotation])

            #Adds nickname, last.
            if not name[1] == "NA" and not name[2] == "NA":
                self.names.append([name[1] + " " + name[2], denotation]) 

            #Adds title, last. 
            if not name[2] == "NA":
                for title in titles:
                    self.names.append([title + " " + name[2], denotation])


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

        print "DENOTATIONS"
        print self.denotations

        self.genSemanticForm(templateType)
        self.genDenotation(templateType)

        return ' '.join(command)

    def genSemanticForm(self, templateType):
        #TODO
        pass


    def genDenotation(self, templateType):
        if templateType == "walk":
            self.denotation = "walk(" + self.denotations["<P>"] + ")"
        elif templateType == "bring":
            self.denotation = "bring("
            self.denotation += self.denotations["<I>"] + ","
            self.denotation += self.denotations["<N>"] + ")"
        elif templateType == "search":
            self.denotation = "searchroom("
            self.denotation += self.denotations["<N>"] + ","
            self.denotation += self.denotations["<P>"] + ")"
            

    def processWord(self, word):
        if word in self.tags:
            expansionIndex = random.randint(0, len(self.tags[word]) - 1)
            expansionList = self.tags[word][expansionIndex].split(':')
            expansion = expansionList[0].strip()
            denotation = expansionList[1].strip()

            #Stores denotation. 
            self.denotations[word] = denotation

            return expansion

        #Expands name from name list of names that was built. 
        elif word == "<N>":
            nameList = self.names[random.randint(0, len(self.names) - 1)]
            name = nameList[0]
            denotation = nameList[1]

            #Stores denotation
            self.denotations[word] = denotation

            return name

        else:
            return word

    def saveData(self):
        phraseFile = open('results/phrases/' + str(self.user_id) + '_' + 'phrase' + '_' + str(self.phrase_num), 'w')
        denotationFile = open('results/denotations/' + str(self.user_id) + '_' + 'denotation' + '_' + str(self.phrase_num), 'w')
        semanticFile = open('results/semantic_forms/' + str(self.user_id) + '_' + 'semantic' + '_' + str(self.phrase_num), 'w')

        #Writes data. 
        phraseFile.write(self.phrase)
        denotationFile.write(self.denotation)
        semanticFile.write(self.denotation) #TODO actually generate semantic form. 

        #Closes file. 
        phraseFile.close()
        denotationFile.close()
        semanticFile.close()

        #Updates phrase number. 
        self.phrase_num += 1

class Recorder:
    def __init__(self, user_id):
        import ctypes

        if True: #TODO Allow path specification is None:
            self.libHandle = ctypes.CDLL("./record.so")
        else:
            self.libHandle = ctypes.CDLL(path)

        self.user_id = user_id
        self.phrase_num = 0
        self.scriptGenerator = None

        #Initialize microphone. 
        self.libHandle.initMic()

    def __del__(self):
        self.libHandle.closeMic()

    def record(self):
        while not self.libHandle.isInterrupted():
            self.libHandle.record1600Hz("results/recordings/" + self.user_id + "_recording_" + str(self.phrase_num))
            self.phrase_num += 1

            print "Recorded something" 

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
        self.scriptGenerator = ScriptGenerator(displayInfo, self.user_id)
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
if not len(sys.argv) == 2:
    print "Usage: python record.py [user_id]"
    sys.exit()

recorder = Recorder(sys.argv[1])
recorder.recordUser()
