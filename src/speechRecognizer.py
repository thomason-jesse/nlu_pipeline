#!/usr/bin/env python
"""
Used for interacting with Sphinx. 
"""

__author__ = 'rodolfo'

import ctypes
import threading
import pygame
import sys
import rospy
import shutil
from std_msgs.msg import String


class InputFromSpeech:
    def __init__(self):
        import ctypes

        self.n = 5

        if True: #TODO Allow path specification is None:
            self.libHandle = ctypes.CDLL("./src/nlu_pipeline/src/speech/speechRecognizer.so")
        else:
            self.libHandle = ctypes.CDLL(path)

        #Initialize Sphinx. 
        self.libHandle.sphinx_init()
        self.libHandle.initMic()

        self.numbers = {"zero": '0', "one": '1', "two": '2', "three": '3', "four": '4', "five": '5',
                        "six": '6', "seven": '7', "eight": '8', "nine": '9'}

    def __del__(self):
        self.libHandle.closeMic()
        self.libHandle.sphinx_close()

    def setN(self, n):
        self.n = n

    def postProcess(self, words):
        utterance = ""

        for word in words:
            if word in self.numbers:
                utterance += self.numbers[word]
            else:
                utterance += ' ' + word + ' '

        return utterance

    def get(self):
        return self.getNBest(self.n)

    def record(self):
        self.libHandle.sphinx_n_best_m(self.n); 

    def recordToggle(self):
        #Initializes pygame window to read spacebar presses. 
        pygame.init()
       
        #Sets up screen size and background color. 
        screen = pygame.display.set_mode((375,90))
        screen.fill((105,105,105))    

        #Initializes font for on-screen instructions. 
        pygame.display.set_caption('Recorder')
        font = pygame.font.SysFont("monospace", 15)
        
        #Writes instructions. 
        text1 = font.render("Click this window to activate recorder", 1, (255,255,255))
        text2 = font.render("Press [SPACEBAR] to record", 1, (255,255,255))
        text3 = font.render("Press [Ctrl-C] to interrupt", 1, (255,255,255))
        

        #Renders instructions on screen. 
        screen.blit(text1, (0,0))
        screen.blit(text2, (0,30))
        screen.blit(text3, (0,60))

        pygame.display.update()

        running = True

        print "\nPress [SPACEBAR] to record"

        while running:
            pygame.event.pump()
            keys = pygame.key.get_pressed()

            if keys[pygame.K_SPACE]:
                print "Recording"

                #Starts recording from mic to file. 
                self.libHandle.startRecord()

                #Loops while user is holding space bar. 
                while keys[pygame.K_SPACE]:
                    pygame.event.pump()
                    keys = pygame.key.get_pressed()

                #Stops recording when they release it. 
                self.libHandle.stopRecord()

                running = False

            if (keys[pygame.K_RCTRL] or keys[pygame.K_LCTRL]) and keys[pygame.K_c]:
                running = False

                self.libHandle.interruptRecord()
                self.libHandle.sphinx_interrupt()

                rospy.signal_shutdown("Interrupted")
        
            #Sleeps to free up come CPU
            pygame.time.wait(100)

        pygame.quit()

    def getNBest(self, n):

        #Creates threads for recording. 
        thread1 = threading.Thread(target = self.record)
        thread2 = threading.Thread(target = self.recordToggle)

        #Starts thread execution. 
        thread1.start()
        thread2.start()

        #Waits for recording threads to finish. 
        thread1.join()
        thread2.join()

        #Checks to see if system was interrupted. 
        if __name__ == '__main__' and rospy.is_shutdown():
            sys.exit()

        #Opens up results to begin reading from them. 
        results = open("results.txt", "r")

        #Makes a list of each hypothesis, containing string, confidence, etc.
        resultList = [line.strip('\n') for line in results]

        hypotheses = [result.split(":") for result in resultList]

        #Stores raw audio and recognition result pair.  
        #self.storeData()

        return hypotheses

    def getHypString(self, hypothesis):
        return hypothesis[0]

    def storeData(self):
        name = self.getUniqueName()

if __name__ == '__main__':
    recognizer = InputFromSpeech()
    
    if len(sys.argv) > 1:
        recognizer.setN(int(sys.argv[1]))

    pub = rospy.Publisher('speech', String, queue_size=1)
    rospy.init_node('recognizer', anonymous=True)

    #Continuously records utterances until interrupted. 
    while not rospy.is_shutdown():
        results = recognizer.get()

        result_strings = [recognizer.postProcess(result[1].split()) for result in results]

        result = ';'.join(result_strings)

        rospy.loginfo(results)
        pub.publish(result)
 

