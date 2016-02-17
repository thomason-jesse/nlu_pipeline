import ctypes
import threading
import pygame
import random

class ScriptGenerator:
    def __init__(self, displayInfo):
        #Initializes data structure for generating phrases. 
        self.templates = []
        self.tags = {}
        self.names = []

        self.genTags()
        self.genTemplates()
        self.genNames()

        #Sets font for displaying text. 
        self.font = pygame.font.SysFont('monospace', 15)
        self.textColor = (255, 255, 255)
        self.phrase = None
        self.screen_w = displayInfo.current_w
        self.screen_h = displayInfo.current_h

    def genPhrase(self, screen):
        command = self.genCommand()    

        self.phrase = self.font.render(command, 1, self.textColor)

        screen.blit(self.phrase, self.calcPos())
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


class Recorder:
    def __init__(self):
        import ctypes

        if True: #TODO Allow path specification is None:
            self.libHandle = ctypes.CDLL("./record.so")
        else:
            self.libHandle = ctypes.CDLL(path)

        #Initialize Sphinx. 
        self.libHandle.initMic()

    def __del__(self):
        self.libHandle.closeMic()

    def record(self):
        self.libHandle.record1600Hz()

    def recordToggle(self):
        #Initializes pygame window to read spacebar presses. 
        pygame.init()

        #Makes mouse invisible. 
        pygame.mouse.set_visible(False)

        #Colors to be used for recording. 
        green = (0, 150, 0)
        red = (150, 0, 0)

        #Gets monitor information and uses it to go to fullscreen mode. 
        displayInfo = pygame.display.Info()
        screen = pygame.display.set_mode((displayInfo.current_w, displayInfo.current_h), pygame.FULLSCREEN)

        print "\nPress [SPACEBAR] to record"

        running = True

        #Creates the phrase generating object. 
        scriptGenerator = ScriptGenerator(displayInfo)

        while running:
            pygame.event.pump()
            keys = pygame.key.get_pressed()

            if keys[pygame.K_SPACE]:
                print "Recording"

                #Starts recording from mic to file. 
                self.libHandle.startRecord()

                #Writes phrase on screen. 
                scriptGenerator.genPhrase(screen) 

                #Loops while user is holding space bar. 
                while keys[pygame.K_SPACE]:
                    pygame.event.pump()
                    keys = pygame.key.get_pressed()

                #Waits for user to press spacebar again to stop recording. 
                while running:
                    pygame.event.pump()
                    keys = pygame.key.get_pressed()

                    if keys[pygame.K_SPACE]:
                        running = False

                    #Sleeps to free up cpu. 
                    pygame.time.wait(100)

                #Stops recording when they release it. 
                self.libHandle.stopRecord()

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

recorder = Recorder()
recorder.recordUser()
