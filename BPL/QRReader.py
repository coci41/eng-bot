#!/usr/bin/env python
# coding=utf-8
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from pyzbar.pyzbar import decode

class QRReader:

    def __init__(self):
        # Istanzio l'oggetto CvBridge che mi permettera la conversione da compressed image a un'immagine cv2
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('qr_reader', Int32, queue_size=1)
        self.sub = rospy.Subscriber("image", CompressedImage, self.callback)
        rospy.init_node('qr_reader_node', anonymous=True)
        rate = rospy.Rate(10)

    def callback(self, data):
        try:

            # Converto il messaggio ricevuto
            img = self.bridge.compressed_imgmsg_to_cv2(data)

            cv2.imshow('IMG', img)

            # for barcode in decode(img):
            #     myData = barcode.data.decode('utf-8')
            #     #commento console
            #     print(myData)
            #
            #     myOutput = 'ALT!!!'
            #     myColor = (255, 0, 0)
            #
            #     pts = np.array([barcode.polygon], np.int32)
            #     pts = pts.reshape((-1, 1, 2))
            #     cv2.polylines(img, [pts], True, myColor, 5)
            #     pts2 = barcode.rect
            #     cv2.putText(img, myOutput, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX,
            #                 0.9, myColor, 2)
            #
            #     cv2.imshow('Result', img)
            #
            #     self.pub.publish(1)

        except CvBridgeError as e:
            print(e)

    # Display barcode and QR code location
    def display(im, bbox):
        n = len(bbox)
        for j in range(n):
            cv2.line(im, tuple(bbox[j][0]), tuple(bbox[(j + 1) % n][0]), (255, 0, 0), 3)

        # Display results
        cv2.imshow("Results", im)


def main():
    d = QRReader()
    # rospy.spin() non fa nulla finchè il flag di rospy.is_shutdown() è true
    rospy.spin()


if __name__ == '__main__':
    main()
