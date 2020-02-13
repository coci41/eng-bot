#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
import gpiozero


class RobotMover():

    def __init__(self):

        self.robot = gpiozero.Robot(left=(27, 22, 17), right=(23, 24, 25))
        self.forward_speed = 0.35
        self.turn_speed = 0.3
        self.AVANTI = 'AVANTI'

        self.sub = rospy.Subscriber('robot_mover',String,self.callback)
        rospy.init_node('robot_mover', anonymous=True)

    def callback(self, data):

        if data:
            print(data)
            if data is 'SX':
                self.robot.left(self.turn_speed)
            elif data is 'DX':
                self.robot.right(self.turn_speed)
            elif data is self.AVANTI:
                self.robot.forward(self.forward_speed)
            elif data is 'TOOSMALL':
                self.robot.left(self.turn_speed)
            elif data is 'FOUND':
                self.robot.stop()
            elif data is 'NOTFOUND':
                self.robot.left(self.turn_speed)
        else:
            self.robot.left(self.turn_speed)


def main():
    r = RobotMover()
    rospy.spin()

if __name__ == '__main__':
    main()
