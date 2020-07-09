#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage,Image

class CameraImage:

    # 0 è default, cam principale
    def __init__(self,device=0):
	#Ricevo i frame dal device - apro la camera
        self.dev = cv2.VideoCapture(device)
        self.__devid = device
	#la risoluzione viene settata a 320x240 - a risoluzione bassa la trasmissione era più fluida e non laggava
        self.dev.set(3,320)	
        self.dev.set(4,240)
	#Lancio un nodo ROS che pubblica sul topic "frame" un messaggio di tipo CompressedImage - scelto piuttosto che immagine standard
	#cosi da velocizzare la trasmissione
        self.pub = rospy.Publisher('frame', CompressedImage, queue_size=1)
        rospy.init_node('camera', anonymous=True)
        rate = rospy.Rate(10)

    def setup(self):
        if not self.dev.isOpened():
            self.dev.open(self.__devid)

    def get_image(self, gray=False):
        try:
            while not rospy.is_shutdown():
		        #leggo i frame ricevuti - ret boolean ovvero se l'operazione di lettura va a buon fine, frame è l'immagine
                ret,frame= self.dev.read()
                if ret:
                    if gray:
                        # se lo voglio grigio lo trasformo in scala di grigi
                        frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		            #Converto il frame ricevuto nel tipo CompressedImage che poi verra' pubblicato nel topic
                    frame = CvBridge().cv2_to_compressed_imgmsg(frame, dst_format='jpg')
                    #Pubblico il frame sul topic frame
                    self.pub.publish(frame)
        except rospy.ROSInterruptException, KeyboardInterrupt:
	    #rilascio il device - quindi si spegne la cam
            dev.release()

def main(): 
    #Instanzio l'oggetto della classe CameraImage 
    c = CameraImage()
    c.get_image()


if __name__ == '__main__':
    main()
