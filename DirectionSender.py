#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class DirectionSender():

    def __init__(self):
        self.HUE_VAL = 28
        self.lower_color = np.array([self.HUE_VAL - 10, 100, 100])
        self.upper_color = np.array([self.HUE_VAL + 10, 255, 255])
        self.minimum_area = 300
        self.maximum_area = 90000
        self.image_width = 320
        self.image_height = 240
        self.center_image_x = self.image_width / 2
        self.center_image_y = self.image_height / 2
        self.sub = rospy.Subscriber('frame_topic',CompressedImage,self.callback)
        self.pub = rospy.Publisher('robot_mover', String, queue_size=1)
        rospy.init_node('frame_receiver', anonymous=True)

    def callback(self, data):
        global image
        try:
            image = CvBridge().compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_mask = cv2.inRange(hsv, self.lower_color, self.upper_color)

        image2, countours, hierarchy = cv2.findContours(color_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        object_area = 0
        object_x = 0
        object_y = 0

        for contour in countours:
            x, y, width, height = cv2.boundingRect(contour)
            found_area = width * height
            center_x = x + (width / 2)
            center_y = y + (height / 2)
            if object_area < found_area:
                object_area = found_area
                object_x = center_x
                object_y = center_y

        if object_area > 0:
            data = [object_area, object_x, object_y]
        else:
            data = None

        if data:
            if (data[0] > self.minimum_area) and (data[0] < self.maximum_area):
                if data[1] > (self.center_image_x + (self.image_width / 3)):
                    self.pub.publish("DX")
                elif data[1] < (self.center_image_x - (self.image_width / 3)):
                    self.pub.publish("SX")
                else:
                    self.pub.publish("AVANTI")
            elif (data[0] < self.minimum_area):
                self.pub.publish("TOOSMALL")
            else:
                self.pub.publish("FOUND")
        else:
            self.pub.publish("NOTFOUND")

def main():
    d = DirectionSender()
    rospy.spin()

if __name__ == '__main__':
    main()
