#!/usr/bin/env python
"""
Test file for experimenting with image and pcl data
"""

import rospy
import sensor_msgs.point_cloud2 as pc2
import cv_bridge
import cv2
from sensor_msgs.msg import Image, PointCloud2

bridge = cv_bridge.CvBridge()

def callback_image(data):
    print 'Received image'
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    (rows,cols,channels) = cv_image.shape
    cv2.imshow("Image visualizer", cv_image)
    cv2.waitKey(3)

def callback_pointcloud(data):
    print 'Received point cloud'
    cloud_gen = pc2.read_points(data, skip_nans=True, field_names=("x","y","z"))
    points = list(cloud_gen)
    mins = [10, 10, 10]
    maxs = [0, 0, 0]
    for d in points:
        for i in range(3):
            if d[i] < mins[i]:
                mins[i] = d[i]
            if d[i] > maxs[i]:
                maxs[i] = d[i]    
    rospy.loginfo(mins)
    rospy.loginfo(maxs)
    
def listener():

    rospy.init_node('kinect_listener')

    # rospy.Subscriber('/kinect2/hd/image_color', Image, callback_image)
    rospy.Subscriber('/kinect2/sd/points', PointCloud2, callback_pointcloud)

    rospy.spin()

if __name__ == '__main__':
    listener()
