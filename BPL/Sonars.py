#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Float64
from gpiozero import DistanceSensor
from time import sleep

GPIO.setmode(GPIO.BCM)
#Creo due array in cui l'i-esimo trigger sara' associato all'i-esimo echo
trigger_pins = [20, 12]
echo_pins = [21, 16]


class SonarNode:

    def __init__(self, trigger_pins, echo_pins):
        if len(trigger_pins) != len(echo_pins):
            print("trigger and echo have different length")
            exit(-1)
        self.sonars = []
        self.pubs = []
        # Istanzio un oggetto DistanceSensor per ogni elemento negli array trigger_pins ed echo_pins , creando il relativo publisher
        for i in range(len(trigger_pins)):
            self.sonars.append(DistanceSensor(trigger=trigger_pins[i], echo=echo_pins[i], queue_len=1, partial=True))
            self.pubs.append(rospy.Publisher("/sonar" + str(i), Float64, queue_size=1))

        rospy.init_node('sonars', anonymous=False)
        self.rate = rospy.Rate(30)

    def misuraDistanza(self):
        while True:
            try:
                # Ogni elemento dell'array pubs , pubblica la propria misurazione con una approssimazione alla seconda cifra decimale
                for i in range(len(self.pubs)):
                    self.pubs[i].publish(round(self.sonars[i].distance * 100, 2))
            except rospy.ROSInterruptException:
                pass
            self.rate.sleep()


def main():
    sonars = SonarNode(trigger_pins, echo_pins)
    sonars.misuraDistanza()


if __name__ == '__main__':
    main()
