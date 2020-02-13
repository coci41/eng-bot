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

class Recognition:

    def __init__(self):
        # Viene settata una misura per la distanza critica
        self.DISTANZA_CRITICA = 25
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('ostacolo', Float32, queue_size=1)
        self.sub = rospy.Subscriber("frame", CompressedImage, self.callback)
        rospy.init_node('riconoscimento_ostacolo', anonymous=True)
        rate = rospy.Rate(10)

    def callback(self, data):
        try:

            cv_img = self.bridge.compressed_imgmsg_to_cv2(data)
            # Ipotizzando che l'ostacolo sia di colore verde, vengono settati i seguenti parametri
            # per settare altri colori , viene utilizzata la classe cameraColor con il metodo di setColor
            lower = (31, 159, 110)
            upper = (68, 255, 255)

            color = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            # Settiamo il range di colori da riconoscere
            mask = cv2.inRange(color, lower, upper)
            target = cv2.bitwise_and(cv_img, cv_img, mask=mask)

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
                M = cv2.moments(c)
                if M["m00"] != 0:
                    # coordinate del centroide rispetto l'asse X e Y
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # Traslo le coordinate in modo da settare idealmente il centro dell'immagine in basso al centro (non in alto a sx come da Default)
                    cX_new = cX - 160
                    cY_new = 240 - cY
                    angle = math.atan2(cX_new, cY_new)
                else:
                    cX = -1
                    cY = -1
                    angle = 1000
                # Ritorna a partire dall'area, la distanza
                dis = self.distance_to_camera(area)

                cv2.drawContours(cv_img, [box], -1, (255, 255, 255), 2)
                cv2.circle(cv_img, (cX, cY), 7, (255, 255, 255), -1)

                # Il nodo pubblichera' se e solo se la vicinanza dell'ostacolo e' minore della distanza critica e se e'
                # stato rilevato un oggetto
                if dis < self.DISTANZA_CRITICA and cX != -1 and cY != -1:
                    self.pub.publish(angle * 180 / math.pi)

            cv2.imshow("draw_image", cv_img)
            cv2.waitKey(1)


        except CvBridgeError as e:
            print(e)

    def distance_to_camera(self, x):
        # L'ostacolo viene riconosciuto in base alla funzione definita interpolando dei campioni (Ostacolo,Distanza),
		# e che dunque associa ad un'area una distanza.
        if x != 0:
            return -132.5719 + (131738.3 - -132.5719) / (1 + (x / 1.901842e-16) ** 0.1476739)
        else:
            return 0


def main():
    d = Recognition()
    rospy.spin()


if __name__ == '__main__':
    main()
