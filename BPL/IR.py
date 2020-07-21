#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String, Int32
from time import sleep

GPIO.setmode(GPIO.BCM)


class IR:

    def __init__(self):
        self.pinArray = [26] #codice più manutenibule, se voglio aggiungere IR basta aggiungere nell'array gli altri pin e andrà a creare tanti topic quanti sono gli IR - comportamento dinamico qualora voglia aggiungere altri IR
        if len(self.pinArray) == 0:
            print("NESSUN IR INDIVIDUATO!!!")
            exit(-1)
        # lista di infrarossi
        self.irArray = []
        # lista di topic
        self.publisherArray = []

        for i in range(len(self.pinArray)):
            # setto valore dei PIN in modalità INPUT - CERCARE
            self.irArray.append(GPIO.setup(self.pinArray[i], GPIO.IN))
            self.publisherArray.append(rospy.Publisher("/infrared_" + str(i + 1), Int32, queue_size=1))

        # crea un nodo solo perchè basta un nodo solo che pubblica su tanti topic, non ha senso avere un nodo per ogni infrarosso (vedi esempio sonars)
        rospy.init_node('ir_node', anonymous=False)
        self.rate = rospy.Rate(10)

    def readAndPublishIR(self):
        while True:
            try:
                # Ogni elemento dell'array pubs , pubblica la propria misurazione
                for i in range(len(self.publisherArray)):
                    # pubblica nel topic[i] in valore dell'iesimo pin
                    self.publisherArray[i].publish(GPIO.input(self.pinArray[i]))
                    sleep(0.5)
            except rospy.ROSInterruptException:
                pass
            self.rate.sleep()


def main():
    i = IR()
    i.readAndPublishIR()
    # CERCARE - tipo while true - senza lo fa una volta sola
    rospy.spin()


if __name__ == '__main__':
    main()
