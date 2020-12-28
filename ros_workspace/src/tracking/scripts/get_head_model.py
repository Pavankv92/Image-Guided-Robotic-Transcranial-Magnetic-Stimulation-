#!/usr/bin/env python
"""
Script for getting head model data
"""

import time
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float64MultiArray
from tracking.msg import float_array
import open3d as o3d
import copy

def pc2_to_list(point_cloud2):
    """ Takes a point cloud 2 and outputs the respective list of points."""
    cloud_gen = pc2.read_points(point_cloud2, skip_nans=True, field_names=("x","y","z"))
    return list(cloud_gen)

class ModelServer():

    DEFAULT_THRESHOLDS = [-0.2, 0.2, -0.3, 0.3, 0.7, 1.2]
    ICP_DEFAULT_THRESHOLD = 0.05

    def __init__(self, debug=False):
        """Initializes a ModelServer."""

        # Check Parameters
        if not rospy.has_param('pcl_threshold_x_min'):
            rospy.set_param('pcl_threshold_x_min',
                            ModelServer.DEFAULT_THRESHOLDS[0])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('pcl_threshold_x_max'):
            rospy.set_param('pcl_threshold_x_max',
                            ModelServer.DEFAULT_THRESHOLDS[1])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('pcl_threshold_y_min'):
            rospy.set_param('pcl_threshold_y_min',
                            ModelServer.DEFAULT_THRESHOLDS[2])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('pcl_threshold_y_max'):
            rospy.set_param('pcl_threshold_y_max',
                            ModelServer.DEFAULT_THRESHOLDS[3])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('pcl_threshold_z_min'):
            rospy.set_param('pcl_threshold_z_min',
                            ModelServer.DEFAULT_THRESHOLDS[4])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('pcl_threshold_z_max'):
            rospy.set_param('pcl_threshold_z_max',
                            ModelServer.DEFAULT_THRESHOLDS[5])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('icp_threshold'):
            rospy.set_param('icp_threshold', 
                            ModelServer.ICP_DEFAULT_THRESHOLD)
            rospy.logwarn('Threshold parameter was not set. Using default.')

        self.continue_flag = True
        self.pc_array = list()
        self.point_cloud = o3d.geometry.PointCloud()

        # Init Ros Node
        rospy.init_node('model_server')

    def thresholding(self, points_as_list):
        """Applies thresholding to given pcl list using ros params."""

        # Get Parameters
        pcl_threshold_xmin = rospy.get_param('pcl_threshold_x_min')
        pcl_threshold_xmax = rospy.get_param('pcl_threshold_x_max')
        pcl_threshold_ymin = rospy.get_param('pcl_threshold_y_min')
        pcl_threshold_ymax = rospy.get_param('pcl_threshold_y_max')
        pcl_threshold_zmin = rospy.get_param('pcl_threshold_z_min')
        pcl_threshold_zmax = rospy.get_param('pcl_threshold_z_max')

        result = list()
        for d in points_as_list:
            if d[0] > pcl_threshold_xmin and \
                d[0] < pcl_threshold_xmax and \
                d[1] > pcl_threshold_ymin and \
                d[1] < pcl_threshold_ymax and \
                d[2] > pcl_threshold_zmin and \
                    d[2] < pcl_threshold_zmax:
                result.append(d)
        return result

    @staticmethod
    def draw_registration_result(source):
        source_temp = copy.deepcopy(source)
        source_temp.paint_uniform_color([1, 0.706, 0])
        o3d.visualization.draw_geometries([source_temp]) # pylint: disable=no-member

    def pcl_callback(self, pcl):
        """Callback for the pcl. Input should be a PointCloud2."""
        if self.continue_flag:
            # print 'Received Pointcloud. '
            points_as_list = pc2_to_list(pcl)  # Convert to List
            thresholded = self.thresholding(points_as_list)  # Apply Thresholds

            self.point_cloud = o3d.geometry.PointCloud() # pylint: disable=no-member
            self.point_cloud.points = o3d.utility.Vector3dVector(list(thresholded)) # pylint: disable=no-member
            # print str(len(self.point_cloud.points))

            # DEBUG draw
            # ModelServer.draw_registration_result(self.point_cloud)

            self.pc_array.append(self.point_cloud)
            self.continue_flag = False

    def start(self):
        """Called after init to make sure everything has been set up correctly."""
        exiting = False
        self.pcl_subscriber = rospy.Subscriber(
                '/kinect2/sd/points', PointCloud2, self.pcl_callback)
        while not exiting:
            stt = raw_input('Enter: ')
            if stt.startswith('e'):
                exiting = True
            else:
                self.continue_flag = True
        for i in range(0, len(self.pc_array)):
            o3d.io.write_point_cloud('../data/head_model/head_model_' + str(i) + # pylint: disable=no-member
                '.ply', self.pc_array[i])

if __name__ == '__main__':
    ts = ModelServer()
    print 'Init'
    ts.start()
