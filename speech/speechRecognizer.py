"""
Used for interacting with Sphinx. 
"""

__author__ = 'rodolfo'

import ctypes
import threading
import pygame
import sys

libHandle = None

def init(soPath = None):
    global libHandle

    if soPath is None:
        libHandle = ctypes.CDLL("./speechRecognizer.so")
    else:
        libHandle = ctypes.CDLL(path)

    libHandle.sphinx_init()
    libHandle.initMic(); 

def record():
    libHandle.sphinx_n_best_m(1)

def recordPress():
    pygame.init()
    pygame.display.set_mode((1,1))

    running = True

    print "\nPress [SPACEBAR] to record"

    while running:
        pygame.event.pump()
        keys = pygame.key.get_pressed()

        if keys[pygame.K_SPACE]:
            libHandle.startRecord()

            print "Recording"

            while keys[pygame.K_SPACE]:
                pygame.event.pump()
                keys = pygame.key.get_pressed()

            libHandle.stopRecord()
            pygame.quit()
            running = False


def getNBest(n):
    t1 = threading.Thread(target = record)
    t2 = threading.Thread(target = recordPress)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print "Recognizing..."

    #Opens file with recognition results. 
    results = open("results.txt", "r")

    resultList = [line.strip('\n') for line in results]

    #Makes a list from each formatted hypothesis string. 
    hypotheses = [result.split(":") for result in resultList]

    return hypotheses

init()

for i in range(0,5):
    print getNBest(1)

libHandle.sphinx_close()
