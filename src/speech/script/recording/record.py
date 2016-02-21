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
        self.denotation = None
        self.semantic_form = None

        #Sets font for displaying text. 
        self.font = pygame.font.SysFont('monospace', 15)
        self.textColor = (255, 255, 255)
        self.background = (191, 87, 0)
        self.screen_w = displayInfo.current_w
        self.screen_h = displayInfo.current_h

    def genPhrase(self, screen):
        self.phrase = self.genCommand()    

        phraseText = self.font.render(command, 1, self.textColor)

        screen.fill(self.background)
        screen.blit(phraseText, self.calcPos())
        pygame.display.update()

    def calcPos(self):
        width = self.phrase.get_width()
        height = self.phrase.get_height()

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
        
        #TODO generate denotation and semantic forms!!!

        #Picks a random template from that action's templates. 
        templateIndex = random.randint(0, len(templateType) - 1)
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

    def saveData(self):
        phraseFile = open('results/phrases/' + str(self.user_id) + '_' + 'phrase' + '_' + str(self.phrase_num), 'w')
        denotationFile = open('results/denotations/' + str(self.user_id) + '_' + 'denotation' + '_' + str(self.phrase_num), 'w')
        semanticFile = open('results/semantic_forms/' + str(self.user_id) + '_' + 'semantic' + '_' + str(self.phrase_num), 'w')

        #Writes data. 
        phraseFile.write(self.phrase)
        denotationFile.write(self.denotation)
        semanticFile.write(self.semantic_form)

        #Closes file. 
        phraseFile.close()
        denotationFile.close()
        semanticFile.close()

class Recorder:
    def __init__(self, user_id):
        import ctypes

        if True: #TODO Allow path specification is None:
            self.libHandle = ctypes.CDLL("./record.so")
        else:
            self.libHandle = ctypes.CDLL(path)

        self.user_id = user_id
        self.phraseNum = 0

        #Initialize microphone. 
        self.libHandle.initMic()

    def __del__(self):
        self.libHandle.closeMic()

    def record(self):
        while not self.libHandle.isInterrupted():
            self.libHandle.record1600Hz(self.userId + "_" + str(self.phraseNum))
            self.phraseNum += 1

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
        scriptGenerator = ScriptGenerator(displayInfo, self.user_id)
        scriptGenerator.genPhrase(screen)

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
                scriptGenerator.saveData()

                #Waits for user to get next phrase.     
                while not keys[pygame.K_RIGHT]:
                    pygame.event.pump()
                    keys = pygame.key.get_pressed()

                #Waits for user to let go of key.     
                while keys[pygame.K_RIGHT]:
                    pygame.event.pump()
                    keys = pygame.key.get_pressed()

                scriptGenerator.genPhrase(screen)

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
