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

class Main:

    def __init__(self, device=0):
        GPIO.setmode(GPIO.BCM)
        self.sonarDx = -1
        self.sonarSx = -1
        self.angoloOstacolo = 1000
        self.infrared = -1
        self.angoloDefault = -1
        self.subscribeAll()

        rospy.init_node('main', anonymous=True)
        self.rate = rospy.Rate(60)

    def subscribeAll(self):
        rospy.Subscriber("lane_follower", Float32, self.laneFollower)
        rospy.Subscriber("obstacle_recognition", Float32, self.obstacleAvoidance)
        rospy.Subscriber("infrared_1", Int32, self.getIRValue)
        rospy.Subscriber("sonar_1", Float64, self.getSonarDxValue)
        rospy.Subscriber("sonar_2", Float64, self.getSonarSxValue)

    # ************* FUNZIONI DI CALLBACK *********************
    def laneFollower(self, data):
        self.angoloDefault = data.data

    def getIRValue(self, data):
        self.infrared = data.data

    def obstacleAvoidance(self, data):
        self.angoloOstacolo = data.data

    def getSonarDxValue(self, data):
        self.sonarDx = data.data

    def getSonarSxValue(self, data):
        self.sonarSx = data.data

    # ***************** GUIDA DIFFERENZIALE ***********************
    #la guida differenziale e' stata spiegata ampiamente nella relazione - la X è l'angolo e la m è l'istanza della classe motore
    def guidaDifferenziale(self, X, m):
        v_dx = 49.66585 + 0.01731602 * X + 0.001274749 * (X ** 2) - 0.00004329004 * (X ** 3)
        v_sx = 49.66585 - 0.01731602 * X + 0.001274749 * (X ** 2) + 0.00004329004 * (X ** 3)
        m.forward(dutyCycleDx=v_dx, dutyCycleSx=v_sx)
        return 1

    # nel caso dell'ostacolo inverto i valori di dx e sx e vado in direzione opposta a quella che suggerisce il centroide
    def guidaDifferenzialeOstacolo(self, X, m):
        v_dx = 49.66585 + 0.01731602 * X + 0.001274749 * (X ** 2) - 0.00004329004 * (X ** 3)
        v_sx = 49.66585 - 0.01731602 * X + 0.001274749 * (X ** 2) + 0.00004329004 * (X ** 3)
        m.forward(dutyCycleDx=v_sx, dutyCycleSx=v_dx)
        return 1


def main():

    # istanziamo l'oggetto per muovere i motori
    m = controller.Controller()

    a = Main()

    #******* COMPORTAMENTO REATTIVO DEL ROBOT ********
    # CERCARE rospyshutdown finchè il canale è aperto
    while not rospy.is_shutdown():
         #l'infrarosso ha priorita' maggiore, e' posizionato avanti
        if (a.infrared == 0):
            print("OSTACOLO DAVANTI!!!")
            # va a indietro e con l'ausilio dei sonar si ruota sul lato piu' libero
            m.back(50, 50)
            if a.sonarSx >= a.sonarDx:
                m.forward(30, 50)
            else:
                m.forward(50, 30)

        # se rileva un muro a destra o sinistra si gira leggermente verso la direzione opposta
        elif (a.sonarDx <= 20):
            print("OSTACOLO A DESTRA : ", a.sonarDx)
            m.forward(30, 50)
        elif (a.sonarSx <= 20):
            print("OSTACOLO A SINISTRA : ", a.sonarSx)
            m.forward(50, 30)

        #se riconosce l'ostacolo, tramite l'angolo del centroide, si attiva la guida differenziale per scansare l'ostacolo
        # quando a.ost e' 1000 significa che o non ha rilevato un ostacolo oppure e' troppo distante
        elif a.angoloOstacolo != 1000:
            print("OSTACOLO IDENTIFICATO", a.angoloOstacolo)
            M = a.guidaDifferenzialeOstacolo(a.angoloOstacolo, m)

        #se nessuno dei sensori si attiva, seguendo le linee bianche, prova a raggiungere la coordinata successiva
        else:
            print("SEGUO LA CORSIA")
            M = a.guidaDifferenziale(a.angoloDefault, m)

        time.sleep(0.1)

    GPIO.cleanup()


if __name__ == '__main__':
    main()
