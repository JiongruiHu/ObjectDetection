#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import matplotlib.pyplot as plt




def callback(data):
        #print "data.header: ", data.header
     tfBuffer = tf2_ros.Buffer()
     listener = tf2_ros.TransformListener(tfBuffer)
     trans = tfBuffer.lookup_transform('world','ifm3d/camera_link', rospy.Time(0))
     cloud_world = do_transform_cloud(data, trans)
     cloud_world.header.frame_id = "world"
     cloud_pub.publish(cloud_world)



if __name__ == '__main__':
     rospy.init_node('camera_tf_listener')

     
     rate = rospy.Rate(10)

     cloud_pub = rospy.Publisher('/camera_cloud_world', PointCloud2, queue_size=1, latch= True)
     cloud_sub = rospy.Subscriber('ifm3d/camera/cloud', PointCloud2, callback)
     rospy.spin()