#!/usr/bin/env python
"""
Test file for experimenting with image and pcl data
"""

import rospy
import sensor_msgs.point_cloud2 as pc2
import cv_bridge
import cv2
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct

bridge = cv_bridge.CvBridge()
    
""" def listener():

    publisher = rospy.Publisher('/kinect2/hd/image_color', Image)

    rospy.init_node('image_publisher')

    rate = rospy.Rate(1)

    height = 100
    width = 100
    blank_image = np.zeros((height,width,3), np.uint8)
    blank_image[:,0:width//2] = (255,0,0)      # (B, G, R)
    blank_image[:,width//2:width] = (0,255,0)

    flag = True
    while not rospy.is_shutdown():
        print "Test publish"
        publisher.publish(bridge.cv2_to_imgmsg(blank_image))
        if flag:
            blank_image[:,0:width//2] = (0,0,255)      # (B, G, R)
        else:
            blank_image[:,0:width//2] = (255,0,0)      # (B, G, R)
        flag = not flag
        rate.sleep()     """

def listener():

    publisher = rospy.Publisher('/kinect2/sd/points', PointCloud2)

    rospy.init_node('image_publisher')

    rate = rospy.Rate(1)

    points = []
    lim = 8
    for i in range(lim):
        for j in range(lim):
            for k in range(lim):
                x = float(i) / lim
                y = float(j) / lim
                z = float(k) / lim
                r = int(x * 255.0)
                g = int(y * 255.0)
                b = int(z * 255.0)
                a = 255
                print r, g, b, a
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                print hex(rgb)
                pt = [x, y, z, rgb]
                points.append(pt)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            # PointField('rgb', 12, PointField.UINT32, 1),
            # PointField('rgba', 12, PointField.UINT32, 1),
            ]

    print points

    header = Header()
    header.frame_id = "map"
    cloud = pc2.create_cloud(header, fields, points)

    flag = True
    while not rospy.is_shutdown():
        print "Test publish"
        cloud.header.stamp = rospy.Time.now()
        publisher.publish(cloud)
        rate.sleep()    

if __name__ == '__main__':
    listener()
