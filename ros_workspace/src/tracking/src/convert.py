"""
This file contains conversion functions to map ros data formats to numpy
and opencv for processing and back for transmission.
"""
import sensor_msgs.point_cloud2 as pc2

def pc2_to_list(point_cloud2):
    """ Takes a point cloud 2 and outputs the respective list of points."""

    # print str(point_cloud2.fields)

    # print len(list(pc2.read_points(point_cloud2, field_names=("x","y","z"))))
    # print len(list(pc2.read_points(point_cloud2, field_names=("x","y","z", "rgb"))))
    cloud_gen = pc2.read_points(point_cloud2, field_names=("x","y","z", "rgb"))
    return list(cloud_gen)
