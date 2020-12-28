#!/usr/bin/env python

import rospy
import numpy as np
from forward_kinematics.msg import float_array
from sensor_msgs.msg import JointState
from forward_kinematics.srv import DhToPoseMatrix

def get_endeffector_pos():
    try:
        joint = rospy.wait_for_message('joint_states', JointState, timeout=10)
    except rospy.ROSException:
        rospy.logerr("Can't receive a joint position from the robot driver")
        exit()
    try:
        rospy.wait_for_service('transformation_server', timeout=10)
        transformation_server = rospy.ServiceProxy('transformation_server', DhToPoseMatrix)
    except rospy.ROSException:
        rospy.logerr("I need the transformation server, please make sure it is running")
        exit()
    try:
        resp = transformation_server(joint.position)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: $s"%e)
        exit()
    pose = np.reshape(resp.pose, (4,4))
    rospy.loginfo("I got this endeffector pose:\n" + str(pose) + "\n")
    return pose

def get_head_pos():
    try:
        head_cam = rospy.wait_for_message("/headposition", float_array, timeout=10)
    except rospy.ROSException:
        rospy.logerr("I need to get a head pose on topic '/headposition', please make sure the tracking node is running")
        exit()
    head_cam = np.reshape(head_cam.data, (4,4))
    rospy.loginfo("head in camera: \n")
    rospy.loginfo(head_cam)
    hand_eye = np.load("../data/maty.npy")
    head_robot = np.dot(hand_eye, head_cam)

    rospy.loginfo("I got this head pose:\n"+ str(head_robot)+ "\n")
    return head_robot

def get_offset():
    rospy.loginfo("Asking for positions to compute the offset, the head and robot sould not move right now")
    endeffector = get_endeffector_pos()
    head = get_head_pos()
    off = np.dot(np.linalg.inv(endeffector), head)  #TODO right order?
    rospy.loginfo("The offset is:\n" + str(off) + "\n")
    return off

if __name__ == "__main__":
    rospy.init_node("Offset_generator", anonymous=True)
    get_offset()