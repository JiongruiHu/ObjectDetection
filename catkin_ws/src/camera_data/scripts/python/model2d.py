#!/usr/bin/env python
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge, CvBridgeError
from sklearn.cluster import MeanShift, estimate_bandwidth, KMeans

#import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rospy
import cv2
import boundingbox as BB
import sensor_msgs.point_cloud2 as pc2
import scipy.stats as sts
import numpy as np
import json


class Model2D:
    def __init__(self, id):
        self.object_id = id
        self.box = []   # coordinators of the four corners of the bounding box
        self.width = 0
        self.length = 0
        self.height = 0

        rospy.init_node('object_model')
        rospy.Subscriber('/ifm3d/camera/cloud', PointCloud2, self._pcd_cb)
        rospy.spin()
        # self.amp_sub = rospy.Subscriber('/ifm3d/camera/amplitude', Image, self._amp_cb)

    def _pcd_cb(self, pcd):
        # rospy.loginfo("Processing point cloud data")
        self.box = _build_model(pcd)
        edges = self.box[1:] - self.box[:-1]
        dist = [np.sum(np.square(e), axis=0) for e in edges]
        self.length = np.sqrt(max(dist))
        self.width = np.sqrt(min(dist))
        rospy.signal_shutdown('Quit')

        '''
        loop = 2
        count = len(self.length)

        #self.box = _build_model_2d(pcd)

        #if len(self.box) == 0:
        if count < loop:
            self.box = _build_model_2d(pcd)
            edges = self.box[1:] - self.box[:-1]
            dist = [np.sum(np.square(e), axis=0) for e in edges]
            self.length.append(np.sqrt(max(dist)))
            self.width.append(np.sqrt(min(dist)))
        else:
            self.length = np.mean(self.length)
            self.width = np.mean(self.width)
            rospy.signal_shutdown('Quit')
        '''
    def _amp_cb(self, amplitude):
        rospy.loginfo("info of image amplitude")
        try:
            bridge = CvBridge()
            img_amp = bridge.imgmsg_to_cv2(amplitude, '32FC1')
            img = cv2.resize(img_amp, (0, 0), fx=3, fy=3)
            img_array = np.array(img, dtype=np.dtype('uint8'))
            row, col = img.shape
            if len(self.box) is not 0:
                box = self.box
                print box
                x1, y1 = int(box[0][0]*col), int(box[0][1]*row)
                x2, y2 = int(box[1][0]*col), int(box[1][1]*row)

                x3, y3 = int(box[2][0]*col), int(box[2][1]*row)
                x4, y4 = int(box[3][0]*col), int(box[3][1]*row)
                print (x1, y1), (x2, y2)
                print (x3, y3), (x4, y4)

                # img_rec = cv2.rectangle(img_array, (x1, y1),(x2,y2), (255,0,0), 1)
                cv2.line(img_array, (x1, y1), (x2, y2), (255, 0, 0), 1)
                cv2.line(img_array, (x3, y3), (x4, y4), (255, 0, 0), 1)
                cv2.imshow('Image', img_array)
                cv2.waitKey(100)
        except CvBridgeError as err:
            print err

    def to_pose_array(self, box):
        pose = Pose()
        pose_msg = PoseArray()
        pose_msg.header.frame_id = self.object_id
        pose_msg.header.stamp = rospy.Time.now()
        for corner in box:
            pose.position.x, pose.position.y, pose.position.z = corner[0], corner[1], 0.0
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0.0, 0.0, 0.0, 1.0
            pose_msg.poses.append(pose)
        return pose_msg


def _build_model(pcd, dim='2d'):
    '''
    :param pcd: point cloud data
    :return: four corners of bounding box
    '''

    inlier_array = BB.extract_top_layer(pcd, dim)
    inlier_list = inlier_array.tolist()
    #inlier_array1 = BB.filter_outlier_dbscan(inlier_array, 0.08)
    #np.savetxt('object_data/aquador.dat', inlier_array.tolist())

    if len(inlier_array) > 10:

        box_points = BB.bounding_rectangle(inlier_array)
        orientation = BB.get_orientation(box_points)
        '''
        centroid_of_box = np.mean(box_points, axis=0)
        centroid_of_data = np.mean(inlier_array, axis=0)

        # To get centered and zero rotation data
        plt.figure(1)
        plt.scatter(inlier_array[:, 0], inlier_array[:, 1], s=1,c='r')

        inlier_array = inlier_array - centroid_of_box
        R = np.array([[np.cos(orientation), -np.sin(orientation)], [np.sin(orientation), np.cos(orientation)]])
        print 'R:', R
        inlier_array = np.dot(inlier_array, R)
        new_box_points = BB.bounding_rectangle(inlier_array)
        #np.savetxt('object_data/mastri.dat', inlier_array.tolist())
        centroid_of_box = np.mean(new_box_points, axis=0)
        centroid_of_data = np.mean(inlier_array, axis=0)


        #plt.figure(1)
        plt.scatter(inlier_array[:, 0], inlier_array[:, 1], s=1)
        plt.plot(new_box_points[:, 0], new_box_points[:, 1], 'r-')

        plt.plot(centroid_of_box[0], centroid_of_box[1], marker='o',c='red')
        plt.plot(centroid_of_data[0], centroid_of_data[1], marker='x')
        plt.axis('equal')
        plt.show()


        # only for Aquador!!!!
        # The following bandwidth can be automatically detected using
        '''
        bandwidth = estimate_bandwidth(inlier_array, quantile=0.08)
        ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
        ms.fit(inlier_array)
        labels = ms.labels_
        cluster_centers = ms.cluster_centers_
        labels_unique = np.unique(labels)
        n_clusters_ = len(labels_unique)
        index = ms.predict(inlier_array)
        kmeans = KMeans(n_clusters=n_clusters_).fit_predict(inlier_array)

        print("number of estimated clusters : %d" % n_clusters_)
        fig, ax = plt.subplots()
        for k in range(n_clusters_):
            cluster = inlier_array[np.where(index == k)]
            cluster_list = cluster.tolist()
            for p in cluster_list:
                if distance(p, cluster_centers[k]) >= 0.0295/2+0.0001:

                    inlier_list.remove(p)
            circle = plt.Circle((cluster_centers[k][0], cluster_centers[k][1]), 0.0293/2, color='red', fill=False)
            ax.add_patch(circle)
        inlier_array = np.asarray(inlier_list)
        box_points = BB.bounding_rectangle(inlier_array)
        orientation = BB.get_orientation(box_points)
        center_box = np.mean(box_points, axis=0)
        inlier_array = inlier_array - center_box
        R = np.array([[np.cos(orientation), -np.sin(orientation)], [np.sin(orientation), np.cos(orientation)]])
        print 'R:', R
        inlier_array = np.dot(inlier_array, R)
        new_box_points = BB.bounding_rectangle(inlier_array)
        new_center = np.mean(new_box_points, axis=0)
        print 'new center', new_center
        ax.set_aspect('equal')
        plt.scatter(cluster_centers[:, 0], cluster_centers[:, 1], c='red', marker="x")
        plt.scatter(inlier_array[:, 0], inlier_array[:, 1], c='blue', s=1)
        plt.plot(box_points[:, 0], box_points[:, 1], 'r--')
        plt.plot(center_box[0], center_box[1], marker='o')
        plt.plot(new_box_points[:, 0], new_box_points[:, 1], 'b--')
        plt.plot(new_center[0], new_center[1], marker='x')
        plt.show()
        np.savetxt('object_data/aquador2d.dat', inlier_array.tolist())

        '''
        np.random.seed(0)
        x = 3.0 * np.random.rand(2000)
        y = 2.0 * np.random.rand(2000) - 1
        inside = ((x ** 2 + y ** 2 > 1.0) & ((x - 3) ** 2 + y ** 2 > 1.0) & ((x - 1.5) ** 2 + y ** 2 > 0.09))
        points = np.vstack([x[inside], y[inside]]).T
        edges = BB.alpha_shape(points, alpha=0.25, only_outer=True)
        print type(edges)
        print edges
        plt.figure()
        plt.axis('equal')
        plt.plot(points[:, 0], points[:, 1], '.')
        for i, j in edges:
            plt.plot(points[[i, j], 0], points[[i, j], 1])
        plt.show()


        edges = BB.alpha_shape(inlier_array, alpha=0.01, only_outer=True)
        plt.figure()
        plt.axis('equal')

        #plt.plot(inlier_array[:, 0], inlier_array[:, 1], '.')
        plt.scatter(inlier_array[:, 0], inlier_array[:, 1])

        for i, j in edges:
            plt.plot(inlier_array[[i, j], 0], inlier_array[[i, j], 1])
        plt.show()
        '''
        return new_box_points
        #return True


def distance(p1, p2):
    d = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    return d


if __name__ == '__main__':

    try:
        new_object = Model2D('aquador')
        box = new_object.box
        di = dict(length=new_object.length, width=new_object.width, name=new_object.object_id,
                  bounding=box.tolist(),
                  max_number=int(1.2*0.8/(new_object.length * new_object.width)))
        print json.dumps(di)

        with open('object_data/aquador.json', 'w+') as f:  # writing JSON object
            json.dump(di, f, indent=4)

    except rospy.ROSInterruptException:
        pass




