#!/usr/bin/env python
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge, CvBridgeError
from sklearn.cluster import KMeans,MeanShift, estimate_bandwidth

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

import rospy
from myFunctions import extract_top_layer, bounding_rectangle, get_orientation
import sensor_msgs.point_cloud2 as pc2
import scipy.stats as sts
import numpy as np
import json
import pcl
from camera_data.srv import *
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


from open3d import *


class ObjectModel:
    def __init__(self, obj_name):
        self.object_id = obj_name
        self.box = []   # coordinators of the four corners of the bounding box
        self.width = 0
        self.length = 0
        self.height = 0

        self.height_list = []
        self.width_list = []
        self.length_list = []
        self.box_list =[]
        self.count = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(1)
        self.trans = self.get_transformation('world','ifm3d/camera_link')

        rospy.Subscriber('/ifm3d/camera/cloud', PointCloud2, self.pcd_cb)
        self.plane_client = rospy.ServiceProxy('/plane_detection', PlaneDetection)

        #rospy.spin()
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if len(self.height_list) > 10:
                rospy.signal_shutdown('Quit')
            else:
                r.sleep()

        self.height = np.mean(self.height_list)
        self.width = np.mean(self.width_list)
        self.length = np.mean(self.length_list)
        self.box = np.mean(np.asarray(self.box_list), 0)


    def plane_detection_client(self, data):
        rospy.wait_for_service('plane_detection')
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

    def pcd_cb(self, pcd):
        pcd_world = do_transform_cloud(pcd, self.trans)
        pcd_world.header.frame_id = "world"



        box_corners, size = self.build_model(pcd_world, self.object_id)
        if 0 not in size:
            self.length_list.append(size[0])
            self.width_list.append(size[1])
            self.height_list.append(size[2])
            self.box_list.append(box_corners)

        #rospy.signal_shutdown('Quit')

    '''
    def to_pose_array(self, fourcorners):
        pose = Pose()
        pose_msg = PoseArray()
        pose_msg.header.frame_id = self.object_id
        pose_msg.header.stamp = rospy.Time.now()
        for corner in fourcorners:
            pose.position.x, pose.position.y, pose.position.z = corner[0], corner[1], 0.0
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0.0, 0.0, 0.0, 1.0
            pose_msg.poses.append(pose)
        return pose_msg
    '''


    def build_model(self, pcd, obj_name):
        """
        :param pcd: point cloud data
        :return: four corners of the bounding box and the predicted lenght, width, height
        """

        p_header = std_msgs.msg.Header()
        p_header.frame_id = pcd.header.frame_id
        p_header.stamp = rospy.Time.now()

        object_array, floor_array = extract_top_layer(pcd, getObjectHeight=True)
        print "process object data"
        object_msg = pc2.create_cloud_xyz32(p_header, object_array.tolist())
        obj_data = self.plane_detection_client(object_msg)

        print "process floor data"
        p_header.stamp = rospy.Time.now()
        floor_msg = pc2.create_cloud_xyz32(p_header, floor_array.tolist())
        floor_data = self.plane_detection_client(floor_msg)

        floor_list = list(pc2.read_points(floor_data.plane_cloud, skip_nans=False, field_names=("x", "y")))
        #plt.figure(9)
        #plt.scatter(np.asarray(floor_list)[:,0], np.asarray(floor_list)[:,1], s=1)
        #plt.show()

        height = abs(floor_data.height + obj_data.height)

        inlier_list = list(pc2.read_points(obj_data.plane_cloud, skip_nans=False, field_names=("x", "y")))

        inlier_array = np.asarray(inlier_list)

        if len(inlier_array) > 100:  # if there are enough data

            # calculate the size of the object
            box_points = bounding_rectangle(inlier_array)
            edges = box_points[1:] - box_points[:-1]
            dist = [np.sum(np.square(e), axis=0) for e in edges]
            length = np.sqrt(max(dist))
            width = np.sqrt(min(dist))
            orientation = get_orientation(box_points)
            centroid_of_box = np.mean(box_points, axis=0)

            centered_inlier_array = inlier_array - centroid_of_box
            R = np.array([[np.cos(orientation), -np.sin(orientation)], [np.sin(orientation), np.cos(orientation)]])
            new_inlier_array = np.dot(centered_inlier_array, R)

            if obj_name == 'aquador':
                # only for Aquador!!!!
                bandwidth = estimate_bandwidth(inlier_array, quantile=0.08)
                ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
                ms.fit(inlier_array)
                labels = ms.labels_
                cluster_centers = ms.cluster_centers_
                labels_unique = np.unique(labels)
                n_clusters = len(labels_unique)
                index = ms.predict(inlier_array)
                print("number of estimated clusters : %d" % n_clusters)

                #kmeans = KMeans(n_clusters=12, init= 'kmeans++').fit(inlier_array)
                #n_clusters =12

                fig, ax = plt.subplots()
                for k in range(n_clusters):
                    cluster = inlier_array[np.where(index == k)]
                    cluster_list = cluster.tolist()
                    for p in cluster_list:
                        if distance(p, cluster_centers[k]) >= 0.0295/2+0.0001:

                            inlier_list.remove(p)
                    #circle = plt.Circle((cluster_centers[k][0], cluster_centers[k][1]), 0.0293/2, color='red', fill=False)
                    #ax.add_patch(circle)

                inlier_array = np.asarray(inlier_list)
                box_points = bounding_rectangle(inlier_array)
                orientation = get_orientation(box_points)
                center_box = np.mean(box_points, axis=0)
                centered_inlier_array = inlier_array - center_box
                R = np.array([[np.cos(orientation), -np.sin(orientation)], [np.sin(orientation), np.cos(orientation)]])
                new_inlier_array = np.dot(centered_inlier_array, R)
                new_box_points = bounding_rectangle(new_inlier_array)
                new_center = np.mean(new_box_points, axis=0)
                print 'new center', new_center
                # display

                ax.set_aspect('equal')
                plt.scatter(cluster_centers[:, 0], cluster_centers[:, 1], c='red', marker="x")
                plt.scatter(new_inlier_array[:, 0], new_inlier_array[:, 1], c='blue', s=1)
                plt.scatter(inlier_array[:, 0], inlier_array[:, 1], c='red', s=1)

                plt.plot(box_points[:, 0], box_points[:, 1], 'r--')
                plt.plot(center_box[0], center_box[1], marker='o')
                plt.plot(new_box_points[:, 0], new_box_points[:, 1], 'b--')
                plt.plot(new_center[0], new_center[1], marker='x')
                plt.show()
                p = pcl.PointCloud()
                object_template = np.hstack(
                    (new_inlier_array, np.zeros((new_inlier_array.shape[0], 1), dtype=new_inlier_array.dtype)))
                p.from_array(object_template.astype(np.float32))
                p.to_file("/home/jiongrui/catkin_ws/src/object_data/%s.pcd" % obj_name)

            # only for the Aquador
            else:
                p = pcl.PointCloud()
                object_template = np.hstack(
                    (new_inlier_array, np.zeros((new_inlier_array.shape[0], 1), dtype=new_inlier_array.dtype)))
                p.from_array(object_template.astype(np.float32))
                p.to_file("/home/jiongrui/catkin_ws/src/object_data/%s.pcd" % obj_name)
            return box_points, [length, width, height]


def distance(p1, p2):
    d = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    return d



if __name__ == '__main__':

    try:
        choice = raw_input("Please choose product. 1: Aquador 2: Pepsi 3: Box \n Your choice: ")
        if choice =='1':
            name = 'aquador'
        elif choice == '2':
            name = 'pepsi'
        elif choice == '3':
            name = 'box'
        else:
            print "Can't recongnize your choice!"
            quit()
        rospy.init_node('object_model')

        newObject = ObjectModel(name)
        box = newObject.box
        print 'L=', newObject.length, 'W=', newObject.width, 'H=', newObject.height

        di = dict(length=newObject.length, width=newObject.width, height=newObject.height,
                  name=newObject.object_id, boundingbox=newObject.box.tolist(),
                  max_number=int(1.2*0.8/(newObject.length * newObject.width)))
        #print json.dumps(di)

        #with open('/home/jiongrui/catkin_ws/src//object_data/%s.json' % name, 'w+') as f:  # writing JSON object
        #    json.dump(di, f, indent=4)

    except rospy.ROSInterruptException:
        pass
