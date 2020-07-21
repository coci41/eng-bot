#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

#questa classe legge dal topic 'frame', una volta elaborato con il range HSV desiderato, pubblica nel topic 'ostacolo'
# l'angolo del centroide solo nel caso in cui l'ostacolo riconosciuto supera una certa distanza.

class ObstacleRecognition:

    def __init__(self):
        # Viene settata una misura per la distanza critica - 25 dovrebbe essere la distanza dall'oggetto
        self.DISTANZA_CRITICA = 25
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('obstacle_recognition', Float32, queue_size=1)
        self.sub = rospy.Subscriber("image", CompressedImage, self.callback)
        rospy.init_node('obstacle_recognition_node', anonymous=True)
        rate = rospy.Rate(10)

    def callback(self, data):
        try:

            cv_img = self.bridge.compressed_imgmsg_to_cv2(data)
            # Ipotizzando che l'ostacolo sia di colore verde, vengono settati i seguenti parametri
            # per settare altri colori , viene utilizzata la classe cameraColor con il metodo di setColor
            lower = (31, 159, 110)
            upper = (68, 255, 255)

            # trasforma in HSV
            color = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            # Settiamo il range di colori da riconoscere
            mask = cv2.inRange(color, lower, upper)
            # applica all'immagine la maschera in AND e trova solo le parti comuni - ti restituisce solo le parti verdi
            target = cv2.bitwise_and(cv_img, cv_img, mask=mask)

            # trasforma in bianco e nero - PROVA?
            gray = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY)
            cv2.imshow('IMG', gray)
            # Prendiamo tutti i contorni di tutti gli ostacoli che riconosce
            (_, target, _) = cv2.findContours(gray.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            #se viene riconosciuto almeno un contorno
            if len(target):

                # Prendiamo l'ostacolo con l'area piu' grande, quindi il piu vicino
                c = max(target, key=cv2.contourArea)
                # Prendiamo i punti della bounding box piu piccola che ricopre l'ostacolo
                d = cv2.minAreaRect(c)
                box = np.int0(cv2.boxPoints(d))
                # Calcoliamo l'area della bounding box
                area = cv2.contourArea(box)
                moments = cv2.moments(c)
                if moments["m00"] != 0:
                    # coordinate del centroide rispetto l'asse X e Y
                    centroideX = int(moments["m10"] / moments["m00"])
                    centroideY = int(moments["m01"] / moments["m00"])
                    # Traslo le coordinate in modo da settare idealmente il centro dell'immagine in basso al centro (non in alto a sx come da Default)
                    X_trasl = centroideX - 160
                    Y_trasl = 240 - centroideY
                    angle = math.atan2(X_trasl, Y_trasl)
                else:
                    centroideX = -1
                    centroideY = -1
                    angle = 1000
                # Ritorna a partire dall'area, la distanza
                dis = self.distanceToCamera(area)

                cv2.drawContours(cv_img, [box], -1, (255, 255, 255), 2)
                cv2.circle(cv_img, (centroideX, centroideY), 7, (255, 255, 255), -1)

                # Il nodo pubblichera' se e solo se la vicinanza dell'ostacolo e' minore della distanza critica e se e'
                # stato rilevato un oggetto
                if dis < self.DISTANZA_CRITICA and centroideX != -1 and centroideY != -1:
                    self.pub.publish(angle * 180 / math.pi)

            cv2.imshow("draw_image", cv_img)
            cv2.waitKey(1)


        except CvBridgeError as e:
            print(e)

    def distanceToCamera(self, x):
        # L'ostacolo viene riconosciuto in base alla funzione definita interpolando dei campioni (Ostacolo,Distanza), sarebbe (Area, Distanza)
		# e che dunque associa ad un'area una distanza.

        # CERCARE - distanza in base all'area
        # mappo sperimentalmente quanto misura l'area a distanze diverse, creo una funzione e questa sotto è la funzione che pessa per quei punti
        if x != 0:
            return -132.5719 + (131738.3 - -132.5719) / (1 + (x / 1.901842e-16) ** 0.1476739)
        else:
            return 0


def main():
    d = ObstacleRecognition()
    rospy.spin()


if __name__ == '__main__':
    main()
