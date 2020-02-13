#!/usr/bin/env python

import cv2
import numpy
import rospy
import time
import motor as mot
import RPi.GPIO as GPIO
import csv
from std_msgs.msg import Float64, Int32, Float32, Float32MultiArray
from datetime import date

class Azione:

    def __init__(self, device=0):
        GPIO.setmode(GPIO.BCM)
        self.sonar_dx = -1
        self.sonar_sx = -1
        self.ost = 1000
        self.ir_0 = -1
        self.default = -1
        self.def_sub = rospy.Subscriber("seguiLinea", Float32, self.comportamentoDefault)
        self.ost_sub = rospy.Subscriber("ostacolo", Float32, self.riconoscimentoOstacolo)
        self.ir0_sub = rospy.Subscriber("ir0", Int32, self.lettura_ir26)
        self.sonar_sub = rospy.Subscriber("sonar0", Float64, self.lettura_sonar_dx)
        self.sonar_sub = rospy.Subscriber("sonar1", Float64, self.lettura_sonar_sx)

        rospy.init_node('main', anonymous=True)
        self.rate = rospy.Rate(60)

    # ************* FUNZIONI DI CALLBACK *********************
    def comportamentoDefault(self, data):
        self.default = data.data

    def lettura_ir26(self, data):
        self.ir_0 = data.data

    def riconoscimentoOstacolo(self, data):
        self.ost = data.data

    def lettura_sonar_dx(self, data):
        self.sonar_dx = data.data

    def lettura_sonar_sx(self, data):
        self.sonar_sx = data.data

    # ***************** GUIDA DIFFERENZIALE ***********************
    #la guida differenziale e' stata spiegata ampiamente nella relazione
    def movimentoDefault(self, X, m):
        v_dx = 49.66585 + 0.01731602 * X + 0.001274749 * (X ** 2) - 0.00004329004 * (X ** 3)
        v_sx = 49.66585 - 0.01731602 * X + 0.001274749 * (X ** 2) + 0.00004329004 * (X ** 3)
        m.avanti(duty_cycle_dx=v_dx, duty_cycle_sx=v_sx)
        return 1

    def movimentoOstacolo(self, X, m):
        v_dx = 49.66585 + 0.01731602 * X + 0.001274749 * (X ** 2) - 0.00004329004 * (X ** 3)
        v_sx = 49.66585 - 0.01731602 * X + 0.001274749 * (X ** 2) + 0.00004329004 * (X ** 3)
        m.avanti(duty_cycle_dx=v_sx, duty_cycle_sx=v_dx)
        return 1


def main():

    # istanziamo l'oggetto per muovere i motori
    m = mot.motor()

    a = Azione()

    #******* COMPORTAMENTO REATTIVO DEL ROBOT ********
    while not rospy.is_shutdown():
         #l'infrarosso ha priorita' maggiore, e' posizionato avanti
        if (a.ir_0 == 0):
            print("Muro Davanti")
            # va a indietro e con l'ausilio dei sonar si ruota sul lato piu' libero
            m.indietro(50,50)
            if a.sonar_sx >= a.sonar_dx:
                m.avanti(30,50)
            else:
                m.avanti(50, 30)

        # se rileva un muro a destra o sinistra si gira leggermente verso la direzione opposta
        elif (a.sonar_dx <= 20):
            print("Muro a Destra : ", a.sonar_dx)
            m.avanti(30,50)
        elif (a.sonar_sx <= 20):
            print("Muro a Sinistra", a.sonar_sx)
            m.avanti(50,30)

        #se riconosce l'ostacolo, tramite l'angolo del centroide, si attiva la guida differenziale per scansare l'ostacolo
        # quando a.ost e' 1000 significa che o non ha rilevato un ostacolo oppure e' troppo distante
        elif a.ost != 1000:
            print("Ostacolo riconosciuto", a.ost)
            M = a.movimentoOstacolo(a.ost, m)

        #se nessuno dei sensori si attiva, seguendo le linee bianche, prova a raggiungere la coordinata successiva
        else:
            print()
            # vai verso path[next_point] #vai verso il punto successivo
            # if (path[next_point] e' stato raggiunto):   #per raggiunto intendiamo entra in un range dentro la coordinata
                #next_point += 1
            # M = a.movimentoDefault(a.default,m)

        time.sleep(0.1)

    GPIO.cleanup()


if __name__ == '__main__':
    main()
