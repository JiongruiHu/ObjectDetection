#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2

class readCloud:
    def __init__(self):
        self.sub = rospy.Subscriber("ifm3d/camera/cloud", PointCloud2,self.call_back, queue_size = 2**24)
    
    def call_back(self, data):
        rospy.loginfo("reading the point cloud")
        cloud = pc2.read_points(data, skip_nans=False, field_names=("x", "y"))
        


if __name__ == '__main__':
    readCloud()