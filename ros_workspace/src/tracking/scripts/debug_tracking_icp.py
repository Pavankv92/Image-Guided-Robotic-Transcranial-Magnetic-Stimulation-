#!/usr/bin/env python
"""
Test file for trying out icp tracking
"""

import time
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

tracker = None

class Tracker():

    # DEBUG: these are the xmin, xmax, ymin, ymax, zmin, zmax thresholds
    # DEBUG: thresholds are in metres
    THRESHOLDS = (-1, 1, -1, 1, 0.5, 1.5)

    def vizualize(self):
        # TODO write visualization here
        pass

    def apply_thresholds(self, point_list):
        result = list()
        for d in point_list:
            if d[0] > Tracker.THRESHOLDS[0] and \
                d[0] < Tracker.THRESHOLDS[1] and \
                d[1] > Tracker.THRESHOLDS[2] and \
                d[1] < Tracker.THRESHOLDS[3] and \
                d[2] > Tracker.THRESHOLDS[4] and \
                d[2] < Tracker.THRESHOLDS[5]:
                result.append(d)
        return result

    def callback_pointcloud(self, data):
        print 'Received point cloud'
        cloud_gen = pc2.read_points(data, skip_nans=True, field_names=("x","y","z"))
        points = list(cloud_gen)
        if self.mode == 1:
            self.first = self.apply_thresholds(points)
            self.mode = 2
        if self.mode == 3:
            self.second = self.apply_thresholds(points)
            self.mode = 4
        
    def __init__(self):
        self.mode = 0
        self.first = None
        self.second = None
        rospy.init_node('kinect_listener')
        rospy.Subscriber('/kinect2/sd/points', PointCloud2, self.callback_pointcloud)
        input('Press any key to take first image.')
        self.mode = 1
        while self.mode < 3:
            input('Press any key to take second image.')
            if self.mode == 2:
                self.mode = 3
        while self.mode < 4:
            time.sleep(0.1)
        # TODO apply the icp here and print/visualize transform
        rospy.spin()

class ThresholdTest():

    def apply_thresholds(self, point_list):
        result = list()
        for d in point_list:
            if d[0] > Tracker.THRESHOLDS[0] and \
                d[0] < Tracker.THRESHOLDS[1] and \
                d[1] > Tracker.THRESHOLDS[2] and \
                d[1] < Tracker.THRESHOLDS[3] and \
                d[2] > Tracker.THRESHOLDS[4] and \
                d[2] < Tracker.THRESHOLDS[5]:
                result.append(d)
        return result

    def callback_pointcloud(self, data):
        print 'Received point cloud'
        cloud_gen = pc2.read_points(data, skip_nans=True, field_names=("x","y","z"))
        points = list(cloud_gen)
        self.points = points
        
    def __init__(self):
        self.points = None
        rospy.init_node('kinect_listener')
        rospy.Subscriber('/kinect2/sd/points', PointCloud2, self.callback_pointcloud)
        
        publisher = rospy.Publisher('/thresholded', PointCloud2)
        rate = rospy.Rate(5)

        flag = True
        while not rospy.is_shutdown():
            print "Test publish"
            if self.points:
                fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    # PointField('rgb', 12, PointField.UINT32, 1),
                    # PointField('rgba', 12, PointField.UINT32, 1),
                    ]
                header = Header()
                header.frame_id = "map"
                cloud = pc2.create_cloud(header, fields, self.apply_thresholds(self.points))
                cloud.header.stamp = rospy.Time.now()
                publisher.publish(cloud)
            rate.sleep()    

if __name__ == '__main__':
    #Tracker()
    ThresholdTest()