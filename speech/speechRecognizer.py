"""
Used for interacting with Sphinx. 
"""

__author__ = 'rodolfo'

import ctypes
import pygame
import threading

libHandle = None

def init(soPath = None):
    global libHandle

    if soPath is None:
        libHandle = ctypes.CDLL("./speechRecognizer.so")
    else:
        libHandle = ctypes.CDLL(path)

def record():
    libHandle.record1600Hz("voice.raw"); 

def recordToggle():
    #Initializes pygame window to read spacebar presses. 
    pygame.init()
    pygame.display.set_mode(1,1)

    running = True

    while running:
        pygame.event.pump()
        keys = pygame.key.get_pressed()

        if keys[pygame.K_SPACE]:
            print "Recording"

            #Starts recording from mic to file. 
            libHandle.startRecord()

            #Loops while user is holding space bar. 
            while keys[pygame.K_SPACE]:
                pygame.event.pump()
                keys = pygame.key.get_pressed()

            #Stops recording when they release it. 
            libHandle.stopRecord()

            #Shuts down pygame and exits thread
            pygame.quit()
            running = False

def getNBest(n):

    #Creates threads for recording. 
    thread1 = threading.Thread(target = record)
    thread2 = threading.Thread(target = recordToggle)

    #Waits for recording threads to finish. 
    thread1.join()
    thread2.join()

    print "Recognizing..."

    #Attempts to recognize recording. 
    libHandle.sphinx_n_best("voice.raw", n)

    #Opens up results to begin reading from them. 
    results = open("results.txt", "r")

    #Makes a list of each hypothesis, containing string, confidence, etc.
    resultList = [line.strip('\n') for line in results]
    hypotheses = [result.split(":") for result in resultList]

    return hypothesis

def getHypString(hypothesis):
    return hypothesis[0]
