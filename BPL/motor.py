'''
 In questo codice viene implementata la classe motor che va a definire i metodi che verranno utilizzati per il movimento del robot
'''

import RPi.GPIO as GPIO
from time import sleep

in1_dx = 24
in2_dx = 23
en_dx = 25

in1_sx = 27
in2_sx = 22
en_sx = 17


class motor:

    def __init__(self):

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(in1_dx, GPIO.OUT)
        GPIO.setup(in2_dx, GPIO.OUT)
        GPIO.setup(en_dx, GPIO.OUT)
        GPIO.output(in1_dx, GPIO.LOW)
        GPIO.output(in2_dx, GPIO.LOW)

        GPIO.setup(in1_sx, GPIO.OUT)
        GPIO.setup(in2_sx, GPIO.OUT)
        GPIO.setup(en_sx, GPIO.OUT)
        GPIO.output(in1_sx, GPIO.LOW)
        GPIO.output(in2_sx, GPIO.LOW)

        self.p_dx = GPIO.PWM(en_dx, 1000)
        self.p_sx = GPIO.PWM(en_sx, 1000)

        self.p_sx.start(25)
        self.p_dx.start(28)

    def stop(self, duty_cycle=None):

        if duty_cycle != None:
            self.p_dx.ChangeDutyCycle(duty_cycle)
            self.p_sx.ChangeDutyCycle(duty_cycle)

        GPIO.output(in1_dx, GPIO.LOW)
        GPIO.output(in2_dx, GPIO.LOW)
        GPIO.output(in1_sx, GPIO.LOW)
        GPIO.output(in2_sx, GPIO.LOW)

    def indietro(self, duty_cycle_dx=None, duty_cycle_sx=None):

        if duty_cycle_dx != None and duty_cycle_sx != None:
            self.p_dx.ChangeDutyCycle(duty_cycle_dx)
            self.p_sx.ChangeDutyCycle(duty_cycle_sx)

        GPIO.output(in1_dx, GPIO.LOW)
        GPIO.output(in2_dx, GPIO.HIGH)
        GPIO.output(in1_sx, GPIO.LOW)
        GPIO.output(in2_sx, GPIO.HIGH)

    def avanti(self, duty_cycle_dx=None, duty_cycle_sx=None):

        if duty_cycle_dx != None and duty_cycle_sx != None:
            self.p_dx.ChangeDutyCycle(duty_cycle_dx)
            self.p_sx.ChangeDutyCycle(duty_cycle_sx)

        GPIO.output(in1_sx, GPIO.HIGH)
        GPIO.output(in2_sx, GPIO.LOW)
        GPIO.output(in1_dx, GPIO.HIGH)
        GPIO.output(in2_dx, GPIO.LOW)
