#!/usr/bin/env python
# coding=utf-8

import cv2
import numpy
import rospy
import time
import Controller as controller
import RPi.GPIO as GPIO
import csv
from std_msgs.msg import Float64, Int32, Float32, Float32MultiArray
from datetime import date
import gpiozero

class Main:

    def __init__(self, device=0):
        # GPIO.BCM significa che ci riferendo ai pin con il numero "Broadcom SOC channel", questi sono i numeri dopo "GPIO"
        GPIO.setmode(GPIO.BCM)
        self.sonarDx = -1
        self.sonarSx = -1
        self.angoloOstacolo = None
        self.infrared = -1
        self.qr = -1
        self.angoloDefault = -1
        self.robot = gpiozero.Robot(left=(24, 23, 25), right=(22, 27, 17))
        self.turn_speed = 0.3
        self.subscribeAll()

        rospy.init_node('main', anonymous=True)
        self.rate = rospy.Rate(60)

    def subscribeAll(self):
        rospy.Subscriber("lane_follower", Float32, self.laneFollower)
        rospy.Subscriber("obstacle_recognition", Float32, self.obstacleAvoidance)
        rospy.Subscriber("infrared_1", Int32, self.getIRValue)
        rospy.Subscriber("sonar_1", Float64, self.getSonarDxValue)
        rospy.Subscriber("sonar_2", Float64, self.getSonarSxValue)
        rospy.Subscriber("qr_reader", Int32, self.getQRValue)

    def laneFollower(self, data):
        self.angoloDefault = data.data

    def getIRValue(self, data):
        self.infrared = data.data

    def getQRValue(self, data):
        self.qr = data.data

    def obstacleAvoidance(self, data):
        self.angoloOstacolo = data.data

    def getSonarDxValue(self, data):
        self.sonarDx = data.data

    def getSonarSxValue(self, data):
        self.sonarSx = data.data

    #angle è l'angolo di sterzata - c è l'istanza della classe Controller
    def guidaDifferenziale(self, angle, c):
        v_dx = 49.66585 + 0.01731602 * angle + 0.001274749 * (angle ** 2) - 0.00004329004 * (angle ** 3)
        v_sx = 49.66585 - 0.01731602 * angle + 0.001274749 * (angle ** 2) + 0.00004329004 * (angle ** 3)
        c.forward(dutyCycleDx=v_dx, dutyCycleSx=v_sx)
        return 1

    # se incontro un ostacolo inverto i dutyCycle in modo da andare nella direzione opposta a quella del centroide
    # Il duty cycle è la frazione di tempo che un'entità passa in uno stato attivo in proporzione al tempo totale considerato
    def guidaDifferenzialeOstacolo(self, angle, c):
        v_dx = 49.66585 + 0.01731602 * angle + 0.001274749 * (angle ** 2) - 0.00004329004 * (angle ** 3)
        v_sx = 49.66585 - 0.01731602 * angle + 0.001274749 * (angle ** 2) + 0.00004329004 * (angle ** 3)
        c.forward(dutyCycleDx=v_sx, dutyCycleSx=v_dx)
        return 1


def main():

    c = controller.Controller()

    a = Main()

    # COMPORTAMENTO REATTIVO
    # (i comportamenti a livello più alto hanno priorità su quelli a livello più basso e prendono il controllo)
    while not rospy.is_shutdown():
        # PRIORITA' 0 - SENSO DI MARCIA ERRATO
        if (a.qr == 1):
            print("STO ANDANDO AL CONTRARIO!!!")
            print("INIZIO A GIRARE A DESTRA!")
            start = time.time()
            while time.time() < start + 5:
                c.forward(50,1)
            print("FINE WHILE TIME")

        # PRIORITA' 1 - INFRAROSSO DAVANTI
        if (a.infrared == 0):
            print("OSTACOLO DAVANTI!!!")
            c.back(50, 50)
            # scelgo la direzione in base al lato più libero
            if a.sonarSx >= a.sonarDx:
                c.forward(30, 50)
            else:
                c.forward(50, 30)

        # PRIORITA' 2 - OSTACOLI LATERALI - RILEVAMENTO SONAR
        elif (a.sonarDx <= 20):
            # se rilevo un ostacolo a DX vado a SX e viceversa
            print("OSTACOLO A DESTRA : ", a.sonarDx)
            c.forward(30, 50)
        elif (a.sonarSx <= 20):
            print("OSTACOLO A SINISTRA : ", a.sonarSx)
            c.forward(50, 30)

        # PRIORITA' 3 - RICONOSCIMENTO OSTACOLO - CAMERA
        # al rilevamento dell'ostacolo attivo guidaDifferenzialeOstacolo()
        elif a.angoloOstacolo != None:
            print("OSTACOLO IDENTIFICATO", a.angoloOstacolo)
            a.guidaDifferenzialeOstacolo(a.angoloOstacolo, c)
            a.angoloOstacolo = None

        #PRIORITA' 4 - RICONOSCIMENTO CORSIE - CAMERA
        else:
            print("SEGUO LA CORSIA")
            a.guidaDifferenziale(a.angoloDefault, c)

        time.sleep(0.1)

    GPIO.cleanup()


if __name__ == '__main__':
    main()
