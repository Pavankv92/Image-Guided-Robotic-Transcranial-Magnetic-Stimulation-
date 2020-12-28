#!/usr/bin/env python

import rospy
import numpy as np
from forward_kinematics.msg import float_array
from offset import get_offset

class Transformer(object):
    def __init__(self):
        self.head_cam = np.zeros((4,4))
        self.head_base = np.zeros((4,4))
        self.last_pose = np.zeros((4,4))
        self.robot = np.zeros((4,4))
        self.hand_eye = np.load("../data/maty.npy")

        rospy.init_node("camera_to_robot")
        self.offset = get_offset()

        self.pub = rospy.Publisher("target_position", float_array, queue_size=5)
        rospy.Subscriber("/headposition", float_array, self.transform)
        
        rospy.spin()

    def transform(self, msg):
        self.head_cam = np.reshape(msg.data, (4,4))
        self.head_base = np.dot(self.hand_eye, self.head_cam) 
        self.pos_robot = np.dot(self.head_base, np.linalg.inv(self.offset))

        if (abs(self.pos_robot[0][3]- self.last_pose[0][3]) > 0.002 ) or (abs(self.pos_robot[1][3]- self.last_pose[1][3]) > 0.002) or (abs(self.pos_robot[2][3]- self.last_pose[2][3]) > 0.002):
            self.pub.publish(np.reshape(self.pos_robot, (16)))
            rospy.loginfo(self.pos_robot)
        #rospy.loginfo(self.head_base)
        else: 
            rospy.loginfo("delta to small")
        self.last_pose = self.pos_robot

    
if __name__ == "__main__":
    t = Transformer()