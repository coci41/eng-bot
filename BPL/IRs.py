#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String, Int32
from time import sleep

GPIO.setmode(GPIO.BCM)


class IR:

    def __init__(self):
        self.pins = [6] #codice più manutenibule, se voglio aggiungere IR basta aggiungere nell'array gli altri pin e andrà a creare tanti topic quanti sono gli IR - comportamento dinamico qualora voglia aggiungere altri IR
        #PIN GPIO06 - OUTPUT - Nero
        if len(self.pins) == 0:
            print("No IR available")
            exit(-1)
        # lista di infrarossi
        self.irs = []
        # lista di topic
        self.pubs = []

        for i in range(len(self.pins)):
            # setto valore dei PIN in modalità INPUT - CERCARE
            self.irs.append(GPIO.setup(self.pins[i], GPIO.IN))
            self.pubs.append(rospy.Publisher("/ir" + str(i), Int32, queue_size=1))

        # crea un nodo solo perchè basta un nodo solo che pubblica su tanti topic, non ha senso avere un nodo per ogni infrarosso (vedi esempio sonars)
        rospy.init_node('irs', anonymous=False)
        self.rate = rospy.Rate(10)

    def detect(self):
        while True:
            try:
                # Ogni elemento dell'array pubs , pubblica la propria misurazione
                for i in range(len(self.pubs)):
                    # pubblica nel topic[i] in valore dell'iesimo pin
                    self.pubs[i].publish(GPIO.input(self.pins[i]))
                    sleep(0.5)
            except rospy.ROSInterruptException:
                pass
            self.rate.sleep()


def main():
    i = IR()
    i.detect()
    # CERCARE - tipo while true - senza lo fa una volta sola
    rospy.spin()


if __name__ == '__main__':
    main()
