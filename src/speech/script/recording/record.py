import ctypes
import threading
import random
import sys
import os
import time
import shutil
import socket

class Server:
    def __init__(self):
        self.s1 = socket.socket()
        self.s2 = socket.socket()
        self.host = ''
        self.port1 = 65300
        self.port2 = 65301

        self.addr1 = None
        self.addr2 = None
        self.conn1 = None
        self.conn2 = None

        self.s1.bind((self.host, self.port1))
        self.s2.bind((self.host, self.port2))

        self.waitForConnections()

    def waitForConnections(self, num_connections = 1):
        self.s1.listen(num_connections)
        self.conn1, self.addr1 = self.s1.accept()

        self.s2.listen(num_connections)
        self.conn2, self.addr2 = self.s2.accept()

        print "Server connected to both clients."

    def waitForConfirm(self, confirm_code):
        m1 = self.conn1.recv(1024)
        
        if not m1 == confirm_code:
            print "Confirm code incorrect!" 
            sys.exit()

        m2 = self.conn2.recv(1024)

        if not m2 == confirm_code:
            print "Confirm code incorrect!"
            sys.exit()


    #Sends given message to extra machines. 
    def sendMessage(self, message):
        #Confirmation code. 
        confirm_code = str(random.random())
    
        self.conn1.send(message + '|' + confirm_code)
        self.conn2.send(message + '|' + confirm_code)

        #Waits for confirmation of receipt. 
        self.waitForConfirm(confirm_code)

class Client:
    def __init__(self, host, port):
        self.s = socket.socket()
        self.host = host
        self.port = int(port)

        self.connect()

    def connect(self):
       self.s.connect((self.host, self.port)) 

    def sendConfirm(self, confirm_code):
        self.s.send(confirm_code)

class ScriptGenerator:
    def __init__(self, displayInfo = None, user_id = None, mode = "normal", prefix = None, recording = True):
        #If display is none, then this is a run on one of the extra machines. 
        #i.e. we don't need any data structure initialization. 
        if displayInfo == None:
            return
        else:
            import pygame
    
        #Initializes data structure for generating phrases. 
        self.templates = {}
        self.tags = {}
        self.dists = {} #Probability distributions. 
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

        #Prefix for saving user data. 
        self.prefix = prefix

        #Distribution for actions.
        self.walk_prob = ["walk", 0.3]
        self.search_prob = ["search", 0.2]
        self.bring_prob = ["bring", 0.5] #bring action is split into normal and adjective.

        self.dists["actions"] = [self.walk_prob, self.search_prob, self.bring_prob]

        #Distribution within bring action. 
        self.bring_norm_prob = ["normal", 0.4]
        self.bring_adj_prob = ["adjective", 0.6]

        self.dists["bring"] = [self.bring_norm_prob, self.bring_adj_prob]

        #Seeds random number generator. 
        random.seed(time.time())

        #Variables to hold data to be saved.
        self.user_id = user_id
        self.phrase_num = 0
        self.phrase = None
        self.denotations = {} #Keeps specific word denotations.
        self.denotation = None # keeps denotation for entire phrase. 
        self.semantic_form = None
        self.semantic_data = {}

        #Sets font for displaying text. 
        self.font = pygame.font.SysFont('monospace', 24, True)
        self.textColor = (255, 255, 255)
        self.background = (191, 87, 0)
        self.screen_w = displayInfo.current_w
        self.screen_h = displayInfo.current_h

        #Sets up font for options text.
        self.option_font = pygame.font.SysFont('monospace', 15)
        self.instructions = "OPTIONS: [space_bar = toggle recording] [r = re-record phrase] "
        self.instructions += "[d = delete recording] [right_arrow = move to next phrase]"

    def setPrefix(self, prefix):
        self.prefix = prefix

    def genPhrase(self, screen):
        #Generates the phrase itself. (i.e. the text)
        self.phrase = self.genCommand()    

        #Paints phrase on screen. 
        self.drawPhrase(screen)

    def drawPhrase(self, screen):
        import pygame

        #The text object to render to screen. 
        self.phraseText = self.font.render(self.phrase, 1, self.textColor)
        self.option_text = self.option_font.render(self.instructions, 1, self.textColor)

        #Updates display. 
        screen.fill(self.background)
        screen.blit(self.option_text, (0,0))
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
        self.templates["walk"] = []
        self.templates["search"] = []
        self.templates["bring"] = []

        #Templates are all held in lists.  
        #Each list has a template type and sublist with all pertaining templates. 
        self.templates["walk"].append(["walk", [line for line in open('walkTemplates.txt', 'r')]])
        self.templates["walk"].append(["walk_possessive", [line for line in open('walkPossessiveTemplates.txt', 'r')]])
        self.templates["search"].append(["search", [line for line in open('searchTemplates.txt', 'r')]])
        self.templates["bring"].append(["bring", [line for line in open('bringTemplates.txt', 'r')]])

    #Generates a list of adjectives as well as the determiners that precede each one. 
    def genAdjectives(self):
        for line in self.tags["<A>"]:
            splitList = line.split(':')
            determiner = splitList[1].strip()
            adjective = splitList[0].strip()

            self.adjectives.append([determiner, adjective])

        print "Adjectives: " + str(self.adjectives)

    def genNouns(self):
        for line in self.tags["<NO>"]:
            splitList = line.split(':')
            determiner = splitList[1].strip()
            noun = splitList[0].strip()

            self.nouns.append([determiner, noun])

        print "Nouns: " + str(self.nouns)

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
    

    def sampleFromDist(self, dist):
        sample_f = random.random()
        total = 0.0

        for l in dist:
            #l[1] contains probability for action. 
            total += l[1]

            if sample_f < total:
                #l[0] contains the action. 
                return l[0]

    def genCommand(self):
        #Picks a random action. 
        action = self.sampleFromDist(self.dists["actions"])

        print "ACTION: " + str(action)

        #Picks a random sub action.
        templateTypeList = self.templates[action]

        #Picks a list of templates from the action type's list. 
        templateListIndex = random.randint(0, len(templateTypeList) -1)
        templateType = templateTypeList[templateListIndex][0]
        templateList = templateTypeList[templateListIndex][1] 

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
            if self.mode == "normal":
                return self.genNormalItem()
            
            elif self.mode == "adjective":
                return self.genAdjectiveDescribed()
            
            elif self.mode == "mix":
                #Picks either an adjective described or a regular item. 
                if self.sampleFromDist(self.dists["bring"]) == "adjective":
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
        phraseFile = open(self.prefix + '/phrases/' + str(self.user_id) + '_' + 'phrase_' + str(self.phrase_num), 'w')
        denotationFile = open(self.prefix + '/denotations/' + str(self.user_id) + '_' + 'denotation_' + str(self.phrase_num), 'w')
        semanticFile = open(self.prefix + '/semantic_forms/' + str(self.user_id) + '_' + 'semantic_' + str(self.phrase_num), 'w')

        #Writes data. 
        phraseFile.write(self.phrase)
        denotationFile.write(self.denotation)
        semanticFile.write(self.semantic_form)

        #Copies temp recording file to permanent location.
        shutil.copy("temp_record.raw", self.prefix + '/recordings/' + str(self.user_id) + '_' + 'recording_' + str(self.phrase_num)) 

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

        #Returns data so that it may be sent to other recording machines. 
        return ';'.join([str(self.user_id), str(self.phrase_num - 1), self.phrase, self.denotation, self.semantic_form])

    #Saves data packet received from main machine on extra machine. 
    def saveDataPacket(self, data, prefix):
        dataList = data.split(';')

        #Updates data structures. 
        self.user_id = dataList[0]
        self.phrase_num = int(dataList[1])
        self.phrase = dataList[2]
        self.denotation = dataList[3]
        self.semantic_form = dataList[4]

        #With now mirrored data, saves it as usual. 
        self.saveData()

class Recorder:
    def __init__(self, user_id, mode, prefix):
        import ctypes

        if True: #TODO Allow path specification is None:
            self.libHandle = ctypes.CDLL("./record.so")
        else:
            self.libHandle = ctypes.CDLL(path)

        self.user_id = user_id
        self.mode = mode
        self.prefix = prefix
        self.phrase_num = 0
        self.scriptGenerator = None

        #Initialize microphone. 
        self.libHandle.initMics()

    def __del__(self):
        self.libHandle.closeMics()

    def record(self):
        while not self.libHandle.isInterrupted():
            self.libHandle.record1600Hz("temp_record.raw")

    #Recording method for other machines in recording session. 
    def recordExtra(self, host, port):
        client = Client(host, port)
        scriptGenerator = ScriptGenerator()
        scriptGenerator.setPrefix(self.prefix)
        running = True

        #Waits for messages until told to stop. 
        while running:
            message_params = client.s.recv(1024).split('|')
            message_type = message_params[0]
     
            if message_type == "start":
                self.libHandle.startRecord()

            if message_type == "stop":
                self.libHandle.stopRecord()

            if message_type == "interrupt":
                if len(message_params) > 2:
                    data = message_params[1]
                    scriptGenerator.saveDataPacket(data)

                self.libHandle.interruptRecord()

            if message_type == "data":
                data = message_params[1]

                #Saves data received and sends confirmation to main machine to move forward. 
                scriptGenerator.saveDataPacket(data)
            
            #Sends back confirmation message to main machine of message receipt. 
            client.sendConfirm(message_params[len(message_params) - 1])

    #Recording method for main machine in recording session. 
    def recordToggle(self):
        import pygame

        #Initializes pygame window to read spacebar presses. 
        pygame.init()

        #Makes mouse invisible. 
        pygame.mouse.set_visible(False)

        #Gets monitor information and uses it to go to fullscreen mode. 
        displayInfo = pygame.display.Info()
        screen = pygame.display.set_mode((displayInfo.current_w-100, displayInfo.current_h-100))

        #Information for "recording" circle. 
        red = (255, 0, 0)
        orange = (191, 87, 0)
        circPos = (displayInfo.current_w - 200, 200)

        #Keys that may be pressed in this method. 
        space = [pygame.K_SPACE]
        interrupt = [pygame.K_RCTRL, pygame.K_c]
        right = [pygame.K_RIGHT]
        repeat = [pygame.K_r]
        delete = [pygame.K_d]

        print "\nPress [SPACEBAR] to record"

        running = True

        #Creates the phrase generating object. 
        self.scriptGenerator = ScriptGenerator(displayInfo, self.user_id, self.mode, self.prefix)

        #Creates server object to communicate with other recording machines. 
        server = Server()

        while running:
            #Generates phrase
            self.scriptGenerator.genPhrase(screen)
            inPhrase = True
            
            while inPhrase:
                #Key combinations that may be pressed. 
                keys = [space, interrupt]
                pressed = self.waitForKeys(keys)

                if pressed == interrupt:
                    running = False
                    self.libHandle.interruptRecord()

                    #Sends interrupt message to other machines. 
                    server.sendMessage("interrupt")

                if pressed == space:
                    #Waits for space to be released. 
                    self.waitForRelease(space)

                    #Sends start record message to other machines. 
                    server.sendMessage("start")

                    #Starts recording from mic to file. 
                    self.libHandle.startRecord()

                    #Draws circle to signify recording. 
                    pygame.draw.circle(screen, red, circPos, 20, 0) 
                    pygame.display.update()

                    #Waits for user to press spacebar again to stop recording. 
                    self.waitForKeys([space])

                    #Sends stop message to other machines. 
                    server.sendMessage("stop")

                    #Stops recording when they release it. 
                    self.libHandle.stopRecord()
                    inPhrase = False

                    #Gets rid of recording circle. 
                    self.scriptGenerator.drawPhrase(screen)

                    #Keys that may be pressed at this stage. 
                    keys = [right, repeat, interrupt, delete]
                    pressed = self.waitForKeys(keys)
    
                    #If interrupted, then exits program and saves data. 
                    if pressed == interrupt:
                        data = self.scriptGenerator.saveData()
                        running = False
                        self.libHandle.interruptRecord()

                        #Sends pertaining message to other machines. 
                        server.sendMessage("interrupt" + '|' + data)

                    #Saves data and will now move on to next phrase.  
                    if pressed == right:
                        data = self.scriptGenerator.saveData()

                        #Sends data to other machines to save. 
                        server.sendMessage("data" + '|' + data)

                    #If delete, will not save data and deletes recorded file.  
                    if pressed == delete:
                        server.sendMessage("delete")
                        pass

                    #Goes back to being in phrase to record once more. 
                    if pressed == repeat:
                        server.sendMessage("repeat")
                        inPhrase = True

        #Exits pygame. 
        pygame.quit()

    #Waits for one of inputted keys to be pressed. Returns key that was pressed. 
    def waitForKeys(self, keys):
        #Waits for one of the given keys to be pressed. 
        while True:
            pygame.event.pump()
            pressed = pygame.key.get_pressed()

            #Returns key pressed if there is one. 
            for combo in keys:
                num_pressed = 0

                #Checks that each key in combo was pressed. 
                for key in combo:
                    if pressed[key]:
                        num_pressed += 1

                if num_pressed == len(combo):
                    return combo

            #Releases processor for a little bit. 
            time.sleep(0.001)

    def waitForRelease(self, keys):
        zero_pressed = False
        
        #Loops until keys given are no longer being pressed. 
        while not zero_pressed:
            num_pressed = 0

            pygame.event.pump()
            pressed = pygame.key.get_pressed()

            for key in keys:
                if pressed[key]:
                    num_pressed += 1

            #The given keys have been released. 
            if num_pressed == 0:
                zero_pressed = True

    def recordUser(self, is_server = True, host = None, port = None):
        if is_server:
            #Creates threads for recording. 
            thread1 = threading.Thread(target = self.record)
            thread2 = threading.Thread(target = self.recordToggle)
        else:
            #Creates threads for extra machine recording. 
            thread1 = threading.Thread(target = self.record)
            thread2 = threading.Thread(target = self.recordExtra, args = (host, port))

        #Starts thread execution. 
        thread1.start()
        thread2.start()

        #Waits for recording threads to finish. 
        thread1.join()
        thread2.join()
           

#Ensures that arguments were passed correctly.
if not len(sys.argv) >= 5:
    print "Usage: python record.py [user_id] [mode (normal/mix/adjective)] [record (y/n)] [server (y/n)] [host] [port]"
    sys.exit()

user_id = sys.argv[1]
mode = "mix"
prefix = sys.argv[2]

if not (mode == "normal" or mode == "adjective" or mode == "mix"):
    print "Mode not recognized, correct values: normal, adjective, mix."
    sys.exit()

record_opt = sys.argv[3]
    
if not (record_opt == "y" or record_opt == "n"):
    print "record value must be \'y\' or \'n\'."
    sys.exit()

if record_opt == "y":
    record = True
else:
    record = False

#Sets up recording interface if necessary. 
if record:
    is_server = sys.argv[4]
    recorder = Recorder(user_id, mode, prefix)

    #Determines if current machine is main machine or not. 
    if is_server == 'y':
        recorder.recordUser()
    else:
        host = sys.argv[5]
        port = sys.argv[6]
        recorder.recordUser(False, host, port)

else:
    pass
    #TODO script generator without recording environment. 
