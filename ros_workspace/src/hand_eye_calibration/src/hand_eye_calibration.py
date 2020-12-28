#!/usr/bin/env python

from __future__ import print_function

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from poseEstimation import transform_from_image
from solve_handEyeCalibration import solve
from forward_kinematics.msg import float_array



class Calibration(object):
    def __init__(self):
        rospy.init_node('hand_eye_calibration')

        self.curent_robot_pose = np.zeros((4,4))
        self.posRobot = []
        self.posImage = []
        self.img = Image()
        self.bridge = CvBridge()

        rospy.Subscriber('/kinect2/hd/image_mono', Image, self.update_img)
        rospy.Subscriber('/forward_kinematics', float_array, self.update_posRobot)

    def update_img(self, msg):
        self.img = msg

    def update_posRobot(self,msg):
        pos = np.reshape(msg.data, (4,4))
        self.curent_robot_pose = pos


    def get_posImage(self):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.img)
        except CvBridgeError as e:
            print(e)
            return
        try:
            trans = transform_from_image(cv_image)
        except ValueError as e:
            print(e)
            return
        self.posImage.append(trans)
    
    def get_posRobot(self):
        self.posRobot.append(self.curent_robot_pose)
        #rospy.loginfo(self.curent_robot_pose)

    def record_pose(self):
        self.get_posImage()
        self.get_posRobot()

    def hand_eye(self):
        (X,Y) = solve(self.posRobot, self.posImage, 40)
        rospy.loginfo(X)
        rospy.loginfo(Y)
        np.savetxt("hand_X.txt", X)
        np.savetxt("hand_Y.txt", Y)

if __name__ == "__main__":
    calib = Calibration()
    #TODO better version for key input
    rospy.loginfo("Using 40 positions for the calibration")
    for i in range(40):
        rospy.loginfo("please move the robot to new position")
        raw_input("Press Enter to continue...")
        calib.record_pose()
    rospy.loginfo("I got this robot positions: ")
    rospy.loginfo(calib.posRobot)
    rospy.loginfo("and recoded this image poses")
    rospy.loginfo(calib.posImage)
    calib.hand_eye()