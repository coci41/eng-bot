#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Float64
from gpiozero import DistanceSensor
from time import sleep

GPIO.setmode(GPIO.BCM)
#Creo due array in cui l'i-esimo trigger sara' associato all'i-esimo echo - nei sonar si usano due pin
triggerPinArray = [20, 12]
echoPinArray = [21, 16]


class Sonar:

    def __init__(self, triggerPins, echoPins):
        if len(triggerPins) != len(echoPins):
            print("ERRORE NEL SETUP DEI SONAR!!!")
            exit(-1)
        self.sonarArray = []
        self.publisherArray = []
        # Istanzio un oggetto DistanceSensor per ogni elemento negli array trigger_pins ed echo_pins , creando il relativo publisher
        for i in range(len(triggerPins)):
            self.sonarArray.append(DistanceSensor(trigger=triggerPins[i], echo=echoPins[i], queue_len=1, partial=True))
            self.publisherArray.append(rospy.Publisher("/sonar_" + str(i + 1), Float64, queue_size=1))

        rospy.init_node('sonar_node', anonymous=False)
        self.rate = rospy.Rate(30)

    def getAndPublishDistance(self):
        while True:
            try:
                # Ogni elemento dell'array pubs , pubblica la propria misurazione con una approssimazione alla seconda cifra decimale
                for i in range(len(self.publisherArray)):
                    self.publisherArray[i].publish(round(self.sonarArray[i].distance * 100, 2))
            except rospy.ROSInterruptException:
                pass
            self.rate.sleep()


def main():
    sonars = Sonar(triggerPinArray, echoPinArray)
    sonars.getAndPublishDistance()


if __name__ == '__main__':
    main()
