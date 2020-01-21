#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
import gpiozero


class RobotMover():

    def __init__(self):

        self.robot = gpiozero.Robot(left=(24, 23, 25), right=(22, 27, 17))
        self.forward_speed = 0.35
        self.turn_speed = 0.3

        self.sub = rospy.Subscriber('robot_mover',String,self.callback)
        rospy.init_node('robot_mover', anonymous=True)

    def callback(self, data):

        if data:
            print(data)
            if data == 'SX':
                self.robot.left(self.turn_speed)
            elif data == 'DX':
                self.robot.right(self.turn_speed)
            elif data == 'AVANTI':
                self.robot.forward(self.forward_speed)
            elif data == 'TOOSMALL':
                self.robot.left(self.turn_speed)
            elif data == 'FOUND':
                self.robot.stop()
            elif data == 'NOTFOUND':
                self.robot.left(self.turn_speed)
        else:
            self.robot.left(self.turn_speed)


def main():
    r = RobotMover()
    rospy.spin()

if __name__ == '__main__':
    main()
