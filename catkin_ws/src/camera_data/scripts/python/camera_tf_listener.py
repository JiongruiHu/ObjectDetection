#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import matplotlib.pyplot as plt


class Listener:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(1)
        self.trans = self.get_transformation('world','ifm3d/camera_link')

        self.cloud_pub = rospy.Publisher('/camera_cloud_world', PointCloud2, queue_size=1)
        self.cloud_sub = rospy.Subscriber('ifm3d/camera/cloud', PointCloud2, self.callback, queue_size=1)

        rospy.spin()

    def callback(self, data):
        rospy.loginfo("transform the new data")
        cloud_world = do_transform_cloud(data, self.trans)
        cloud_world.header.frame_id = "world"
        #cloud_world.header.stamp = rospy.Time.now
        self.cloud_pub.publish(cloud_world)
        self.rate.sleep()
           

    def get_transformation(self, target_frame, source_frame):
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(target_frame,source_frame, rospy.Time(0))
                
                return trans
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            
        '''
        while not rospy.is_shutdown():
            try:
                #print "data.header: ", data.header
                trans = self.tfBuffer.lookup_transform('world','ifm3d/camera_link', rospy.Time())
                cloud_world = do_transform_cloud(data, trans)
                cloud_world.header.frame_id = "world"
                cloud_world.header.stamp = rospy.Time()
                self.cloud_pub.publish(cloud_world)
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep
                print "something wrong"
                continue
        '''


if __name__ == '__main__':
    rospy.init_node('camera_tf_listener')
    myListener = Listener()
    rospy.spin()