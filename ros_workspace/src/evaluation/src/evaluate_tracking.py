#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib
matplotlib.use('GTKAgg')
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from live_plot import Live_plot
from forward_kinematics.msg import float_array


class plot_speed():
    #this just as a test
    def __init__(self):
        self.robotSpeed = 0
        self.plot = Live_plot()
        self.count = 0
        rospy.Subscriber('/joint_states', JointState, self.update_speed)
        rospy.init_node("speed_evaluation", anonymous=True)
        rospy.spin()

    def update_speed(self, msg):
        if self.count % 20 == 0:
            speed = msg.velocity
            self.robotSpeed = np.amax(speed)
            rospy.loginfo(self.robotSpeed)
            self.plot.update(self.robotSpeed)
        self.count += 1


class evaluate_tracking():
    def __init__(self):
        self.plot = Live_plot()
        rospy.init_node("tracking_evaluation", anonymous=True)
        rospy.loginfo('Init complete, call function')

    def eval_standing(self):
        raw_ref = rospy.wait_for_message('/headposition', float_array)
        self.ref = np.reshape(raw_ref.data, (4,4))
        rospy.Subscriber('/headposition', float_array, self._update_standing)
        rospy.loginfo('Plotting the distance to the ref position')
        rospy.spin()
    
    def _update_standing(self, msg):
        pos = np.reshape(msg.data, (4,4))
        delta = np.linalg.norm((pos[:,3] - self.ref[:,3]), 2)
        rospy.loginfo(delta)
        self.plot.update(delta)

if __name__ == "__main__":
    e = evaluate_tracking()
    plt.xlabel("Timestep")
    plt.ylabel("Distance to reference position")
    e.eval_standing()
    plt.show()