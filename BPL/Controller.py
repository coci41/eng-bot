'''
 In questo codice viene implementata la classe motor che va a definire i metodi che verranno utilizzati per il movimento del robot
'''

import RPi.GPIO as GPIO
from time import sleep

inputSx1 = 23
inputSx2 = 24
enableSx = 25

inputDx1 = 27
inputDx2 = 22
enableDx = 17

class Controller:

    def __init__(self):

        self.initializeGPIO()

        #CERCARE PWN (funzione del motore - slide Chella - potenza del segnale - segnale può essere alto o basso, non ci sono valori intermedi)
        self.powerDx = GPIO.PWM(enableDx, 1000)
        self.powerSx = GPIO.PWM(enableSx, 1000)

        self.powerSx.start(25)
        self.powerDx.start(28)

    def initializeGPIO(self):
        GPIO.setmode(GPIO.BCM)
        # CERCARE GPIO.OUT e SETUP
        GPIO.setup(inputDx1, GPIO.OUT)
        GPIO.setup(inputDx2, GPIO.OUT)
        GPIO.setup(enableDx, GPIO.OUT)
        GPIO.output(inputDx1, GPIO.LOW)
        GPIO.output(inputDx2, GPIO.LOW)
        GPIO.setup(inputSx1, GPIO.OUT)
        GPIO.setup(inputSx2, GPIO.OUT)
        GPIO.setup(enableSx, GPIO.OUT)
        GPIO.output(inputSx1, GPIO.LOW)
        GPIO.output(inputSx2, GPIO.LOW)

    def forward(self, dutyCycleDx=None, dutyCycleSx=None):

        if dutyCycleDx != None and dutyCycleSx != None:
            self.powerDx.ChangeDutyCycle(dutyCycleDx)
            self.powerSx.ChangeDutyCycle(dutyCycleSx)

        GPIO.output(inputSx1, GPIO.HIGH)
        GPIO.output(inputSx2, GPIO.LOW)
        GPIO.output(inputDx1, GPIO.HIGH)
        GPIO.output(inputDx2, GPIO.LOW)

    def back(self, dutyCycleDx=None, dutyCycleSx=None):

        # CERCARE ChangeDutyCycle
        # imposto prima la potenza del motore con ChangeDutyCycle e poi gli dico se andare avanti o indietro
        if dutyCycleDx != None and dutyCycleSx != None:
            self.powerDx.ChangeDutyCycle(dutyCycleDx)
            self.powerSx.ChangeDutyCycle(dutyCycleSx)

        GPIO.output(inputDx1, GPIO.LOW)
        GPIO.output(inputDx2, GPIO.HIGH)
        GPIO.output(inputSx1, GPIO.LOW)
        GPIO.output(inputSx2, GPIO.HIGH)

    def stop(self, dutyCycle=None):

        # CERCARE dutycycle (effettiva potenza del motore)
        if dutyCycle != None:
            self.powerDx.ChangeDutyCycle(dutyCycle)
            self.powerSx.ChangeDutyCycle(dutyCycle)

        GPIO.output(inputDx1, GPIO.LOW)
        GPIO.output(inputDx2, GPIO.LOW)
        GPIO.output(inputSx1, GPIO.LOW)
        GPIO.output(inputSx2, GPIO.LOW)
