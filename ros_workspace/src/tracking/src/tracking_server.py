#!/usr/bin/env python
"""
Tracking server contains all basic tracking functionality
"""

import time
import struct
import binascii
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float64MultiArray
from tracking.msg import float_array
import open3d as o3d
import copy
from sensor_msgs.msg import JointState
from convert import pc2_to_list


class TrackingServer():

    DEFAULT_THRESHOLDS = [-10, 10, -10, 10, 0, 10]
    ICP_DEFAULT_THRESHOLD = 0.05
    RED_VALUE_THRESHOLD = 100
    GREEN_VALUE_THRESHOLD = 70
    BLUE_VALUE_THRESHOLD = 70

    def __init__(self, debug=False):
        """Initializes a TrackingServer."""

        # Open3D Log
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug) # pylint: disable=no-member

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
        self.started = False
        self.head_pos = None
        self.rate_counter = 0 # For rate limiting
        self.transformation = np.asarray([[1.0, 0, 0, 0],
                             [0, 1.0, 0, 0],
                             [0, 0, 1.0, 0], 
                             [0.0, 0.0, 0.0, 1.0]])
        self.pcl_reference = None # pylint: disable=no-member
        self.position_reference = None
        self.pc_new = o3d.geometry.PointCloud() # pylint: disable=no-member
        self.registration_method = o3d.registration.TransformationEstimationPointToPlane() # pylint: disable=no-member

        # Evaluation variables
        self.calculation_times = list()
        self.positions = list()

        rospy.Subscriber('joint_states', JointState, self.ur5_pos)
        self.last_ur5_pos = np.zeros(6)
        self.ur5_pos_a = []

        # Init Ros Node
        rospy.init_node('tracking_server')

        # Init Thresholded Publisher
        self.threshold_publisher = rospy.Publisher(
            '/thresholded', PointCloud2, queue_size=1)

        # Init Position Publisher
        self.position_publisher = rospy.Publisher(
            '/headposition', float_array, queue_size=1)

    def ur5_pos(self, msg):
        self.last_ur5_pos= msg.position

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

                    # Colors
                    color = struct.unpack('>BBBB', bytearray(struct.pack("f", d[3])))
                    # argb = struct.unpack('<B', color[0])
                    # print str(argb)

                    if color[0] == 0 and color[1] == 0 and color[2] == 0:
                        pass
                    elif color[2] >= TrackingServer.RED_VALUE_THRESHOLD and \
                        color[1] <= TrackingServer.GREEN_VALUE_THRESHOLD and \
                        color[0] <= TrackingServer.BLUE_VALUE_THRESHOLD:
                        result.append((d[0], d[1], d[2]))
    
        return result

    @staticmethod
    def draw_registration_result(source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp]) # pylint: disable=no-member

    def calc_head_transform_icp(self, thresholded_points):
        """This is the main function for registering the head position
        transformation. It uses ICP and the open3d library to compare
        the last and newest point clouds (stored in old_points and 
        new_points respectively) and calculates the transform between
        them which is output as a numpy 4x4 matrix.
        """
        calctime = time.time()
        self.pc_new.points = o3d.utility.Vector3dVector(thresholded_points) # pylint: disable=no-member
        
        #print 'Vector3D conversion time: ' + str(time.time() - calctime)
        calctime = time.time()
        #print 'Number of points before: ' + str(len(self.pc_new.points))
        #self.pc_new = self.pc_new.voxel_down_sample(0.005)
        self.pc_new.estimate_normals(fast_normal_computation=False)
        #print 'Downsampling conversion time: ' + str(time.time() - calctime)
        calctime = time.time()
        #print 'Number of points after 1: ' + str(len(self.pc_new.points))
        #self.pc_new = self.pc_new.remove_statistical_outlier(3, 0.1)[0]
        #print 'Statistical outlier conversion time: ' + str(time.time() - calctime)
        calctime = time.time()
        #print 'Number of points after 2: ' + str(len(self.pc_new.points))

        # DEBUG Draw
        # TrackingServer.draw_registration_result(self.pc_old, self.pc_new, trans_init)

        # Apply point to plane ICP
        reg_p2l = o3d.registration.registration_icp( # pylint: disable=no-member
            self.pcl_reference, self.pc_new, 
            rospy.get_param('icp_threshold'), 
            self.transformation,
            self.registration_method,
            o3d.registration.ICPConvergenceCriteria( # pylint: disable=no-member
                relative_fitness=0.0001, 
                relative_rmse=0.0001, 
                max_iteration=30))
        #print 'Algorithm time: ' + str(time.time() - calctime)
        calctime = time.time()
        #print reg_p2l
        #if reg_p2l.fitness < 0.0001:
        #    TrackingServer.draw_registration_result(self.pc_new, self.pcl_reference, self.transformation)

        # NOTE values are better without this currently
        self.transformation = reg_p2l.transformation

        return reg_p2l.transformation

    def pcl_callback(self, pcl):
        """Callback for the pcl. Input should be a PointCloud2."""
        self.rate_counter += 1
        if self.rate_counter % 3 == 0:
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

            if self.started:

                if self.pcl_reference == None:
                    self.pcl_reference = o3d.geometry.PointCloud() # pylint: disable=no-member
                    self.pcl_reference.points = o3d.utility.Vector3dVector(thresholded) # pylint: disable=no-member
                    self.pcl_reference.estimate_normals(fast_normal_computation=False)
                    # Store reference point position
                    pcd_tree = o3d.geometry.KDTreeFlann(self.pcl_reference) # pylint: disable=no-member
                    ref_point = None
                    ref_point_nb = 0
                    # print len(self.pcl_reference.points)
                    for i in range(0, len(self.pcl_reference.points)):
                        nb = pcd_tree.search_radius_vector_3d(self.pcl_reference.points[i], 0.01)
                        if nb > ref_point_nb:
                            ref_point = self.pcl_reference.points[i]
                            ref_point_nb = nb
                            # print i
                    self.position_reference = np.asarray([[1.0, 0, 0, ref_point[0]],
                                [0, 1.0, 0, ref_point[1]],
                                [0, 0, 1.0, ref_point[2]], 
                                [0.0, 0.0, 0.0, 1.0]])
                    # print(self.position_reference)
                    position_shaped = self.position_reference.reshape((16))
                    self.position_publisher.publish(position_shaped.tolist())
                    # self.started = False
                else:
                    calctime = time.time()
                    
                    # Calculate diff from ICP
                    head_tf = self.calc_head_transform_icp(thresholded)
                    #print np.round(head_tf, decimals=4)

                    # Calculate position
                    # if not self.head_pos == None: # DEBUG PRINT
                    #     print (np.dot(self.position_reference, np.linalg.inv(head_tf)) - self.head_pos)
                    #self.head_pos = np.dot(self.position_reference, np.linalg.inv(head_tf))
                    self.head_pos = np.dot(self.position_reference, head_tf)

                    #print 'Calculation finished ' + str((time.time() - calctime))
                    print np.round(self.head_pos, decimals=4)
                    #print "Reference:"
                    #print np.round(self.position_reference, decimals=4)
                    head_tf_shaped = self.head_pos.reshape((16)) # Reshape
                    self.position_publisher.publish(head_tf_shaped.tolist()) # Publish
                    # Eval
                    self.calculation_times.append((time.time() - calctime))
                    self.positions.append(self.head_pos)
                    self.ur5_pos_a.append(self.last_ur5_pos)


    def start(self):
        """Called after init to make sure everything has been set up correctly."""
        self.pcl_subscriber = rospy.Subscriber(
            '/kinect2/sd/points', PointCloud2, self.pcl_callback)
        while not raw_input('Write start to start: ').startswith('start'):
            time.sleep(1)
        self.started = True
        while not raw_input('Write stop: ').startswith('stop'):
            time.sleep(1)
        # fil = open('../data/tracking.txt', 'w+')
        # fil.write(str(self.calculation_times))
        # fil.write(str(self.positions))
        # fil.close()
        np.save("../data/pos", self.positions)
        np.save("../data/time", self.calculation_times)
        np.save("../data/ur5", self.ur5_pos_a)
        print("Stop")

if __name__ == '__main__':
    ts = TrackingServer()
    print 'Init'
    ts.start()
