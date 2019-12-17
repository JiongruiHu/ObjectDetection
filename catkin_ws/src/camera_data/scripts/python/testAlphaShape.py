#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from myFunctions import extract_top_layer
from AlphaShape import alpha_shape
import matplotlib.pyplot as plt


def pcd_cb(data):
    points_top_layer, floor_layer = extract_top_layer(data)
    if points_top_layer.shape[1] == 3:
        points_top_layer = points_top_layer[:, [0, 1]]
    edges = alpha_shape(points_top_layer, 1)
    alpha_points = points_top_layer[edges]
    plt.figure(1)
    plt.scatter(points_top_layer[:, 0], points_top_layer[:, 1])
    plt.plot(alpha_points[:,0], alpha_points[:,1], 'r-')
    plt.show()

    print edges


if __name__ == '__main__':
    rospy.init_node('object_model')
    rospy.Subscriber('/ifm3d/camera/cloud', PointCloud2, pcd_cb)
    rospy.spin()
