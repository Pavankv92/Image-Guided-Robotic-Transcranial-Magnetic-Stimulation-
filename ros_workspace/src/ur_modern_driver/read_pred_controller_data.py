#!/usr/bin/env python
#
# Revision $Id$

import time
import rosbag
import os
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import h5py
import scipy.io as sio
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import glob
import math

import struct
_EPS = np.finfo(float).eps * 4.0

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> np.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> np.allclose(M, np.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> np.allclose(M, np.diag([1, -1, -1, 1]))
    True

    """
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def read_rt_tf(trk_rt_tf, child_frame):
    cam_dev_ts = []
    cam_ros_ts = []
    cam_tf = []
    for topic, msg, tStamp in trk_rt_tf:
        if msg.transform_stamped.child_frame_id == child_frame:
            cam_dev_ts.append(msg.rt_stamp.to_sec())
            cam_ros_ts.append(msg.transform_stamped.header.stamp.to_sec())
            tfCurr = msg.transform_stamped.transform
            H = quaternion_matrix([tfCurr.rotation.w, tfCurr.rotation.x, tfCurr.rotation.y, tfCurr.rotation.z])
            H[0:3, 3] = np.array([tfCurr.translation.x, tfCurr.translation.y, tfCurr.translation.z])
            cam_tf.append(H)
    #
    cam_dev_ts = np.array(cam_dev_ts)
    cam_ros_ts = np.array(cam_ros_ts)
    cam_tf = np.array(cam_tf)
    if cam_tf.size != 0:
        cam_tf = np.swapaxes(np.swapaxes(cam_tf, 0, 2), 0, 1)
    return cam_dev_ts, cam_ros_ts, cam_tf

def read_tf(tf, child_frame):
    cam_ros_ts = []
    cam_tf = []
    for topic, msg, tStamp in tf:
        for tidx in range(len(msg.transforms)):
            if msg.transforms[tidx].child_frame_id == child_frame:
                cam_ros_ts.append(msg.transforms[tidx].header.stamp.to_sec())
                tfCurr = msg.transforms[tidx].transform
                H = quaternion_matrix([tfCurr.rotation.w, tfCurr.rotation.x, tfCurr.rotation.y, tfCurr.rotation.z])
                H[0:3, 3] = np.array([tfCurr.translation.x, tfCurr.translation.y, tfCurr.translation.z])
                cam_tf.append(H)
    #
    cam_ros_ts = np.array(cam_ros_ts)
    cam_tf = np.array(cam_tf)
    cam_tf = np.swapaxes(np.swapaxes(cam_tf, 0, 2), 0, 1)
    return cam_ros_ts, cam_tf

dataFolder = '/data/'
allBagFiles = glob.glob(dataFolder+"*.bag")

for bagFileName in allBagFiles:
    csvFolder = bagFileName.split('.')[0]+'/'
    bag = rosbag.Bag(bagFileName)
    trk_rt_tf = bag.read_messages('/trk_rt_tf')
    # head poses from camera
    cam_dev_ts, cam_ros_ts, cam_tf = read_rt_tf(trk_rt_tf, "tracker_headband")
    if cam_tf.size != 0:
        if not os.path.exists(csvFolder):
            os.makedirs(csvFolder)
        cam_tf[0:3, 3, :] = cam_tf[0:3, 3, :] * 1000
        ctf = cam_tf[0:3, :, :].reshape((1, 12, cam_tf.shape[2])).squeeze()
        ctf = np.vstack([ctf, cam_dev_ts, cam_ros_ts])
        np.savetxt(csvFolder+"tracker_headband.csv", ctf.T, delimiter=",")
        # coil poses from camera
        trk_rt_tf = bag.read_messages('/trk_rt_tf')
        cam_dev_ts, cam_ros_ts, cam_tf = read_rt_tf(trk_rt_tf, "tracker_styluswhite")
        if cam_tf.size != 0:
            cam_tf[0:3, 3, :] = cam_tf[0:3, 3, :] * 1000
            ctf = cam_tf[0:3, :, :].reshape((1, 12, cam_tf.shape[2])).squeeze()
            ctf = np.vstack([ctf, cam_dev_ts, cam_ros_ts])
            np.savetxt(csvFolder+"tracker_styluswhite.csv", ctf.T, delimiter=",")
        # robot poses
        trk_rt_tf = bag.read_messages('/ur_rt_tf')
        cam_dev_ts, cam_ros_ts, cam_tf = read_rt_tf(trk_rt_tf, "tool0_controller")
        if cam_tf.size != 0:
            cam_tf[0:3, 3, :] = cam_tf[0:3, 3, :] * 1000
            ctf = cam_tf[0:3, :, :].reshape((1, 12, cam_tf.shape[2])).squeeze()
            ctf = np.vstack([ctf, cam_dev_ts, cam_ros_ts])
            np.savetxt(csvFolder+"robot.csv", ctf.T, delimiter=",")
