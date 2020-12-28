#!/usr/bin/env python
"""
Tracking server contains all basic tracking functionality
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

from convert import pc2_to_list


class TrackingServer():

    DEFAULT_THRESHOLDS = [-10, 10, -10, 10, 0, 10]
    ICP_DEFAULT_THRESHOLD = 0.05

    def __init__(self, debug=False):
        """Initializes a TrackingServer."""

        # Check Parameters
        if not rospy.has_param('pcl_threshold_x_min'):
            rospy.set_param('pcl_threshold_x_min',
                            TrackingServer.DEFAULT_THRESHOLDS[0])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('pcl_threshold_x_max'):
            rospy.set_param('pcl_threshold_x_max',
                            TrackingServer.DEFAULT_THRESHOLDS[1])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('pcl_threshold_y_min'):
            rospy.set_param('pcl_threshold_y_min',
                            TrackingServer.DEFAULT_THRESHOLDS[2])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('pcl_threshold_y_max'):
            rospy.set_param('pcl_threshold_y_max',
                            TrackingServer.DEFAULT_THRESHOLDS[3])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('pcl_threshold_z_min'):
            rospy.set_param('pcl_threshold_z_min',
                            TrackingServer.DEFAULT_THRESHOLDS[4])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('pcl_threshold_z_max'):
            rospy.set_param('pcl_threshold_z_max',
                            TrackingServer.DEFAULT_THRESHOLDS[5])
            rospy.logwarn('Threshold parameter was not set. Using default.')
        if not rospy.has_param('icp_threshold'):
            rospy.set_param('icp_threshold', 
                            TrackingServer.ICP_DEFAULT_THRESHOLD)
            rospy.logwarn('Threshold parameter was not set. Using default.')

        # Init variables
        self.points = None
        self.counter = 0
        self.sum_matrix = np.zeros((4, 4))
        self.reference = # TODO load from file
        self.current_pc = o3d.geometry.PointCloud() # pylint: disable=no-member
        self.registration_method = o3d.registration.TransformationEstimationPointToPlane() # pylint: disable=no-member
        self.trans_estimate = np.asarray([[1.0, 0, 0, 0],
                             [0, 1.0, 0, 0],
                             [0, 0, 1.0, -1.5], 
                             [0.0, 0.0, 0.0, 1.0]])

        # Init Ros Node
        rospy.init_node('tracking_server')

        # Init Thresholded Publisher
        self.threshold_publisher = rospy.Publisher(
            '/thresholded', PointCloud2, queue_size=1)

        # Init Position Publisher
        self.position_publisher = rospy.Publisher(
            '/headposition', float_array, queue_size=1)

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
    def draw_registration_result(source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp]) # pylint: disable=no-member

    def calc_head_transform_icp(self):
        """This is the main function for registering the head position
        transformation. It uses ICP and the open3d library to compare
        the last and newest point clouds (stored in old_points and 
        new_points respectively) and calculates the transform between
        them which is output as a numpy 4x4 matrix.
        """
        calctime = time.time()
        self.current_pc.points = o3d.utility.Vector3dVector(self.points) # pylint: disable=no-member
        print 'Vector3D conversion time: ' + str(time.time() - calctime)

        # DEBUG Draw
        # TrackingServer.draw_registration_result(self.pc_old, self.pc_new, )

        # Apply point to plane ICP
        calctime = time.time()
        reg_p2l = o3d.registration.registration_icp( # pylint: disable=no-member
            self.reference, self.current_pc, 
            rospy.get_param('icp_threshold'), 
            self.trans_estimate,
            self.registration_method,
            o3d.registration.ICPConvergenceCriteria( # pylint: disable=no-member
                relative_fitness=0.0001, 
                relative_rmse=0.0001, 
                max_iteration=30))
        print 'Algorithm time: ' + str(time.time() - calctime)
        calctime = time.time()
        print reg_p2l
        return reg_p2l.transformation

    def pcl_callback(self, pcl):
        """Callback for the pcl. Input should be a PointCloud2."""
        print 'Received Pointcloud. ' + str(time.time())
        points_as_list = pc2_to_list(pcl)  # Convert to List
        thresholded = self.thresholding(points_as_list)  # Apply Thresholds
        # Uniform reduction
        uniform_reduction_factor = 1
        thresholded = list(thresholded[i] for i in xrange(0, len(thresholded), uniform_reduction_factor))

        # DEBUG Test publish thresholded points
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1)]
        header = Header()
        header.frame_id = "map"
        cloud = pc2.create_cloud(header, fields, thresholded)
        cloud.header.stamp = rospy.Time.now()
        self.threshold_publisher.publish(cloud)
        # END Test publish

        # TODO initial head estimation here
        if not self.points:
            pass

        # Update variables
        self.points = thresholded  # Store new points

        # Call registration method
        if self.old_points != None:
            calctime = time.time()
            head_tf = self.calc_head_transform_icp()
            print 'Calculation finished ' + str((time.time() - calctime))
            print np.round(head_tf, decimals=4)
            head_tf_shaped = head_tf.reshape((16,1))[0] # Reshape
            self.position_publisher.publish(head_tf.tolist()) # Publish
            self.sum_matrix += head_tf
            print(str(self.sum_matrix[0][3]) + ' ' + str(self.sum_matrix[1][3]) + ' ' + str(self.sum_matrix[2][3]))

    def start(self):
        """Called after init to make sure everything has been set up correctly."""
        self.pcl_subscriber = rospy.Subscriber(
            '/kinect2/sd/points', PointCloud2, self.pcl_callback)
        rospy.spin()


if __name__ == '__main__':
    ts = TrackingServer()
    print 'Init'
    ts.start()
