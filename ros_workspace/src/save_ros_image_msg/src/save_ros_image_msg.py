#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from time import time

def callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image vindow", cv_image)
    k = cv2.waitKey(10) & 0xFF
    if k == ord('q'):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("quit")
    elif k == ord('s'):
        try:
            ret = cv2.imwrite("../data/calibration_images/image{}.png".format(time()),cv_image) # change filename to image_ir or image_rgb depending on what you record
        except cv2.error as e:
            print(e)
            return
        if ret:
            rospy.loginfo("Image saved")



def listener():
    rospy.init_node('image_saver', anonymous=True)
    rospy.loginfo("use 's' to save an image, 'q' to quit")
    rospy.Subscriber('image_raw', Image, callback) # change ro real topic (/kinect2/hd/image_mono bzw. /kinect2/sd/image_ir)
    rospy.spin()

if __name__ == "__main__":
    listener()
    cv2.destroyAllWindows()