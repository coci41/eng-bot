#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import cv2

class FrameSender():

    def __init__(self, device=0):

        self.camera = cv2.VideoCapture(device) #usare VideoCapture VNC VIEWER
        self.devId = device
        if not self.camera.isOpened():
                self.camera.open(self.devId)
        self.camera.set(3, 320)
        self.camera.set(4, 240)
        self.pub = rospy.Publisher('frame_topic',CompressedImage, queue_size=1)
        self.bridge = CvBridge()
        rospy.init_node('frame_sender', anonymous=True)
        rate = rospy.Rate(10) # 10hz

    def getImage(self):

        while not rospy.is_shutdown():

            (grabbed, frame) = self.camera.read()
            # cv2.imshow("Frame", frame)
            if grabbed:
                rospy.loginfo("FRAME GRABBATO!!!")
            if not grabbed:
                rospy.loginfo("FRAME NON GRABBATO!!!")
            self.pub.publish(self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg'))

if __name__ == '__main__':
    try:
        fm = FrameSender()
        fm.getImage()
    except rospy.ROSInterruptException:
        pass
