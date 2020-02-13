#!/usr/bin/env python

import cv2
import rospy
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

#E' spiegato nel paragrafo della guida differenziale

class ComportamentoDefault:

    def __init__(self):
        # Istanzio l'oggetto CvBridge che mi permettera la conversione da compressed image a un'immagine cv2
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("frame", CompressedImage, self.callback)
        self.pub = rospy.Publisher('seguiLinea', Float32, queue_size=1)
        rospy.init_node('comportamentoDefault', anonymous=True)
        rate = rospy.Rate(10)

    def callback(self, data):
        def nothing(x):
            pass

        try:
            try:
                # Converto il messaggio ricevuto
                frame = self.bridge.compressed_imgmsg_to_cv2(data)
                # Converto l'immagine da bgr in hsv per ottenera un'immagine con una soglia affinche' riconosca le linee nere
                imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                imgThreshold = cv2.inRange(imgHSV, (50, 0, 0, 0), (255, 255, 100, 0))
                cv2.imshow('imgt', imgThreshold)
                _, contours, _ = cv2.findContours(imgThreshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                centre = []
                for c in contours:
                    # Considero le porzioni nere di immagine con area maggiore di 400, per non considerare dettagli irrilevanti
                    if cv2.contourArea(c) > 400:
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            # Effettuo un taglio orizzontale dell'immagine per rilevare solo i centroidi all'interno della pista
                            if cY > 120:
                                (minAreaRect) = cv2.minAreaRect(c)
                                box = cv2.boxPoints(minAreaRect)
                                box = np.int0(box)
                                cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
                                # Costruisco un punto che contenga le coordinate del centroide e lo inserisco nella lista dei centroidi rilevati
                                point = (cX, cY)
                                centre.append(point)
                        else:
                            # Caso in cui non riconosce alcuna area nera
                            cX, cY = 0, 0

                #queste variabili serviranno dopo a fare la media dei centroidi
                sumX = 0
                sumY = 0
                #Iterando, calcolo la media di tutti i centroidi acquisiti
                for i in range(len(centre)):
                    X, Y = centre[i]
                    sumX += X
                    sumY += Y

                if len(centre):
                    X = sumX / len(centre)
                    Y = sumY / len(centre)
                    # Traslo le coordinate in modo da settare idealmente il centro dell'immagine in basso al centro (non in alto a sx come da Default)
                    X_new = X - 160
                    Y_new = 240 - Y
                    # Calcolo l'angolo del centroide rispetto al nuovo sistema di assi
                    angle = math.atan2(X_new, Y_new)
                else:
                    # Caso in cui l'array dei centroidi e' vuoto
                    X = 0
                    Y = 0
                    angle = 0
                # Visualizzo il centroide sull'immagine
                cv2.circle(frame, (X, Y), 5, (255, 255, 255), -1)
                cv2.putText(frame, "centroid", (X - 25, Y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.imshow("Frame", frame)
                # Pubblico l'angolo in gradi sul topic seguiLinea che verra utilizzato per realizzare la guida differenziale									differenziale
                self.pub.publish(angle * 180 / math.pi)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()

            except CvBridgeError as e:
                print(e)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()


def main():
    d = ComportamentoDefault()
    rospy.spin()


if __name__ == '__main__':
    main()
