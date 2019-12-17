#!/usr/bin/env python

#import sys
import matplotlib
matplotlib.use('Agg')

import rospy
import numpy as np
import std_msgs.msg
import pcl
import json
import tf2_ros
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String, Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion,Vector3
from myFunctions import bounding_rectangle, extract_top_layer, clustering, get_orientation
from camera_data.srv import *
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker

class Cluster:
    #def __init__(self, object_name, length, width, height, max_nr_of_objects, box):
    def __init__(self, object_name, length, width, height, max_nr_of_objects):

        rospy.init_node('object_detection', disable_signals=False)
        rospy.loginfo("starting to gather data from camera")
        self.maxK = max_nr_of_objects
        self.object_name = object_name
        self.object_length = length
        self.object_width = width
        self.object_height = height
        self.object_area = length * width
        #self.box = np.asarray(box)
        self.count = 0
        self. center_list = []

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(1)
        self.trans = self.get_transformation('world','ifm3d/camera_link')

        # topics subsribed and publish
        self.pcd_sub = rospy.Subscriber('ifm3d/camera/cloud', PointCloud2, self.pcd_cb, queue_size=1,buff_size= 2**24)
        self.cloud_pub = rospy.Publisher('/camera_cloud_world', PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher('visualization_marker',Marker, queue_size=1)

        # clients
        self.pcd_client = rospy.ServiceProxy('/register_two_point_clouds', RegisterTwoPointClouds)
        self.plane_client = rospy.ServiceProxy('/plane_detection', CheckPlane)

        rospy.spin()
        # topics to publish
        # self.pose_pub = rospy.Publisher("/object/pose", Pose, queue_size=10)

    def register_two_point_clouds_client(self, data):
        rospy.wait_for_service('register_two_point_clouds',100)
        try:
            resp = self.pcd_client(data)
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def plane_detection_client(self, data):
        rospy.wait_for_service('plane_detection', 100)
        try:
            resp = self.plane_client(data)
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_transformation(self, target_frame, source_frame):
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(target_frame,source_frame, rospy.Time(0))
                
                return trans
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

    def make_line_list_marker(self, top_face_points, h, frame = 'world', line_size = 0.005):
        marker = Marker(type = Marker.LINE_LIST,
                        action = Marker.ADD,
                        ns = "basic_shapes",
                        id = 0)
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.pose.position = Point(0,0,0)
        marker.pose.orientation = Quaternion(0,0,0,1)
       
        # calculate the points for the lines, each line needs two points
        marker.points = []
        for i in range(len(top_face_points)):
            p = top_face_points[i]
            if i == 3:
                p_next = top_face_points[0]
            else:
                p_next = top_face_points[i+1]
            # to show the whole box, take way the '''. 
            '''
            marker.points.extend([Point(p[0], p[1], h), 
                                Point(p[0], p[1], h - self.object_height)])
            marker.points.extend([Point(p[0],p[1],h- self.object_height), 
                                Point(p_next[0],p_next[1], h-self.object_height)])
            '''
            # the bounding box for the top face.
            marker.points.extend([Point(p[0],p[1],h), Point(p_next[0], p_next[1], h)])

        
        marker.scale.x = line_size
        marker.scale.y = line_size
        
        # red lines
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0                
        marker.color.a = 1.0
        # show the lines for 5 sec
        marker.lifetime = rospy.Duration(5)
        return marker


    def pcd_cb(self, pcd):

        rospy.loginfo("Processing the camera data")

        print pcd.height
        print pcd.width
        print pcd.is_dense
        cloud_world = do_transform_cloud(pcd, self.trans)
        cloud_world.header.frame_id = "world"
        
        #points_toplayer, floor_points = extract_top_layer(cloud_world, getObjectHeight=True)

        # all the calculations should be in the world frame
        
        # transfer point_toplayer_3d to sensor_msg/PointCloud2 msg to be able to use the services
        if self.object_name == 'aquador': 
            # if it is aquador ,dont use plane detection service.
            points_toplayer, floor_points, top_layer_height = extract_top_layer(cloud_world, 
                                                                getObjectHeight=True, aquador=True)
            #points_toplayer_list = list(points_toplayer)
        else:
            # if not aquador, use plane detection service.
            points_toplayer, floor_points = extract_top_layer(cloud_world, getObjectHeight=True)
            print type(points_toplayer)

            p_header = std_msgs.msg.Header()
            p_header.frame_id = cloud_world.header.frame_id
            p_header.stamp = rospy.Time.now()
            p_cloud = pc2.create_cloud_xyz32(p_header, points_toplayer.tolist())
            plane_cloud_msg = self.plane_detection_client(p_cloud)
            pcl_data = pcl.PointCloud()
            points_toplayer_list = list(pc2.read_points(plane_cloud_msg.plane_cloud, skip_nans=True, field_names=("x", "y", "z")))
            pcl_data.from_list(points_toplayer_list)

            #pcl_data = pcl.PointCloud(points_toplayer.astype(np.float32))

            pcl_data.to_file("/home/jiongrui/catkin_ws/src/Test/build/toplayer_plane.pcd")

               


if __name__ == '__main__':

    choice = raw_input("Please choose product. 1: Aquador 2: Pepsi 3: CardbordBox \n Your choice: ")
    try:
        if choice is '1':
            with open("/home/jiongrui/catkin_ws/src/object_data/aquador.json", "r") as read_file:
                data = json.load(read_file)
                #template_data = pcl.load('/home/jiongrui/catkin_ws/src/camera_data/scripts/python/object_data/aquador.dat')

        elif choice is '2':
            with open("/home/jiongrui/catkin_ws/src/object_data/pepsi.json", "r") as read_file:
                data = json.load(read_file)
                #template_data = np.loadtxt('/home/jiongrui/catkin_ws/src/camera_data/scripts/python/object_data/pepsi.pcd')

        elif choice is '3':
            with open('/home/jiongrui/catkin_ws/src/object_data/box.json', "r") as read_file:
                #template = pcl.load('/home/jiongrui/catkin_ws/src/camera_data/scripts/python/object_data/box.pcd')
                data = json.load(read_file)
                #template = np.loadtxt('object_data/box.pcd')
        else:
            print "Can't recongnize your choice!"
            quit()

        l = data['length']
        w = data['width']
        h = data['height']
        max_nr = data['max_number']
        name = data['name']
        box = data['boundingbox']

        #Cluster(name, l, w,h, max_nr, box)
        Cluster(name, l, w,h, max_nr)


    except rospy.ROSInterruptException:
        pass

