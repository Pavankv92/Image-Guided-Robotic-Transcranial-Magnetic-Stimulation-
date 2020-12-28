#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import rospkg
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import numpy as np
import scipy.io as sio
import socket

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]

client = None
jjAll = None
time_start = None
startJoints = None

def read_head_data(headTrajMatFile, speedSc, ang6_offset):
    global jjAll
    global time_start
    global startJoints
    # numSampsToUse = 200
    headTrajData = sio.loadmat(headTrajMatFile)
    startJoints = headTrajData['jInit'] * pi / 180.
    startJoints[5, 0] = startJoints[5, 0] - ang6_offset
    # jointSpeeds = headTrajData['jjSp']*pi/180.
    # time_intervals = headTrajData['tIntSelSc']
    time_intervals = headTrajData['tIntFull']
    # numTrajPts = headTrajData['numTrajPts']
    jjAll = headTrajData['jjFull'] * pi / 180.
    jjAll[5, :] = jjAll[5, :] - ang6_offset
    # jjAll = jjAll[:,:numSampsToUse]
    # time_intervals = time_intervals[:numSampsToUse-1]*speedSc
    #time_intervals[0, 50:1250] = time_intervals[0, 50:1250] * speedSc
    time_intervals = time_intervals * speedSc
    time_start = np.append([0], time_intervals.cumsum())

def move_head_traj():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [JointTrajectoryPoint(positions=jjAll[:,idx].flatten().tolist(), velocities=[0]*6, time_from_start=rospy.Duration(time_start[idx])) for idx in range(0,jjAll.shape[1])]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_start_pos():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [JointTrajectoryPoint(positions=np.array(joints_pos).flatten().tolist(), velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=startJoints.flatten().tolist(), velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise
#

def move1():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = 2.0
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
        for i in range(10):
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 1
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 1
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 2
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]

        client.send_goal(g)
        time.sleep(3.0)
        print "Interrupting"
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def main():
    global client
    global data_path
    try:
        rospy.init_node("head_mimick", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        rospack = rospkg.RosPack()
        headTrajMatFile = rospy.get_param('~head_traj_mat_file', rospack.get_path('ur_modern_driver')+'/data/headTrajSystematic.mat')
        ask_first = rospy.get_param('~ask_first',True)
        useCtrlMaster = rospy.get_param('~use_ctrl_master',False)
        ip_ctrlMaster = rospy.get_param('~ip_ctrl_master','127.0.0.1')
        port_ctrlMaster = rospy.get_param('~port_ctrl_master',2525)
        numCases = rospy.get_param('~num_cases',1)
        speedSc = rospy.get_param('~speed_scale',1.5)
        ang6_offset = rospy.get_param('~ang6_offset_deg', 30)*pi/180
        read_head_data(headTrajMatFile, speedSc, ang6_offset)
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        #
        if useCtrlMaster:
            ctrlMaster = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            #
            # adjust IP in the driver.txt to the IP of the computer(when  using the
            # simulator make sure that you take the ip of the computer in the virtual % network!)
            ctrlMaster.connect((ip_ctrlMaster, port_ctrlMaster))
        # make sure it is stopped and no other program is running
        print "This program makes the robot move between the following three poses:"
        print str([startJoints[i,0]*180./pi for i in xrange(0,6)])
        if (ask_first):
            print "Please make sure that your robot can move freely between these poses before proceeding!"
            inp = raw_input("Continue? y/n: ")[0]
        else:
            inp = 'y'
        #
        if (inp == 'y'):
            for c in range(numCases):
                t0 = time.time()
                if useCtrlMaster:
                    ctrlMaster.send('start\n')
                move_start_pos()
                move_head_traj()
                t1 = time.time()
                total = t1 - t0
                print "Time taken (case %d): %f" % (c, total)
                #move1()
            #move_repeated()
            #move_disordered()
            #move_interrupt()
        else:
            print "Halting program"
        if useCtrlMaster:
            ctrlMaster.close()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
