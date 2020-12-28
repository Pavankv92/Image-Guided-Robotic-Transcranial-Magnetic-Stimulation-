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

class ModelGenerator():

    def __init__(self, debug=False):
        """Initializes a ModelServer."""
        self.pc_array = list()

    @staticmethod
    def draw_registration_result(source):
        axis = o3d.geometry.PointCloud() # pylint: disable=no-member
        axis_pts = [[0.0, 0.0, 0.0]]
        axis.points = o3d.utility.Vector3dVector(axis_pts) # pylint: disable=no-member
        axis.paint_uniform_color([1, 0, 0])
        source_temp = copy.deepcopy(source)
        source_temp.paint_uniform_color([1, 0.906, 0])
        o3d.visualization.draw_geometries([source_temp, axis]) # pylint: disable=no-member

    def start(self):
        """Called after init to make sure everything has been set up correctly."""
        for i in range(0, 37):
            self.pc_array.append(o3d.io.read_point_cloud('../data/head_model/head_model_' + str(i) + # pylint: disable=no-member
                '.ply'))

        # transform
        trans_x = 0.0
        trans_y = 0.0
        trans_z = -1.1
        rot_x = 0.0
        rot_y = 0.0
        rot_z = 0.0
        for pc in self.pc_array:
            pc.translate(np.array([trans_x, trans_y, trans_z]))
            pc.rotate(np.array([0.0, 0.0, 0.0]))

            pc.uniform_down_sample(every_k_points=500)
            pc.remove_radius_outlier(nb_points=16, radius=0.05)

            ModelGenerator.draw_registration_result(pc)

        # rotate
        for i in range(0, 37):
            self.pc_array[i].rotate(np.array([0, i * np.pi/36, 0]))

        # combine
        self.model_cloud = o3d.geometry.PointCloud() # pylint: disable=no-member
        for pc in self.pc_array:
            self.model_cloud.points = o3d.utility.Vector3dVector( # pylint: disable=no-member
                np.concatenate(
                (self.model_cloud.points, pc.points), axis=0))
        ModelGenerator.draw_registration_result(self.model_cloud)

        # TODO store model and optimize
        # TODO estimate Normals

if __name__ == '__main__':
    ts = ModelGenerator()
    print 'Init'
    ts.start()
