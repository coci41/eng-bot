#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String, Int32
from time import sleep

GPIO.setmode(GPIO.BCM)


class IR:

    def __init__(self):
        self.pins = [26]
        if len(self.pins) == 0:
            print("No IR available")
            exit(-1)
        self.irs = []
        self.pubs = []

        for i in range(len(self.pins)):
            self.irs.append(GPIO.setup(self.pins[i], GPIO.IN))
            self.pubs.append(rospy.Publisher("/ir" + str(i), Int32, queue_size=1))

        rospy.init_node('irs', anonymous=False)
        self.rate = rospy.Rate(10)

    def detect(self):
        while True:
            try:
                # Ogni elemento dell'array pubs , pubblica la propria misurazione con una approssimazione alla seconda cifra decimale
                for i in range(len(self.pubs)):
                    self.pubs[i].publish(GPIO.input(self.pins[i]))
                    sleep(0.5)
            except rospy.ROSInterruptException:
                pass
            self.rate.sleep()


def main():
    i = IR()
    i.detect()
    rospy.spin()


if __name__ == '__main__':
    main()
