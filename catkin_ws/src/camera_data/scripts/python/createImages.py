#!/usr/bin/env python
#from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt


class ImageConverter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_amp_sub = rospy.Subscriber("ifm3d/camera/amplitude", Image, self.amp_callback)
        #self.image_distance_sub = rospy.Subscriber("ifm3d/camera/distance", Image, self.dist_callback)
        #self.image_unit_vector_sub = rospy.Subscriber("ifm3d/camera/unit_vectors", Image, self.unit_callback)


    def unit_callback(self, data):
        try:
            print data.height
            print data.data[1:3]
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC3")
            cv2.namedWindow('Image unit', cv2.WINDOW_NORMAL)
            cv2.imshow("Image unit", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

    def dist_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            cv2.namedWindow('Image distance', cv2.WINDOW_NORMAL)
            cv2.imshow("Image distance", cv_image)
            cv2.resizeWindow('Image distance', 600, 600)
            cv2.imwrite("/catkin_ws/src/Images/boxdist.jpg", cv_image)

            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

    def amp_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "8UC1")
            mtx =np.asarray(([214.2285, 0, 113.6319], [0, 214.3580,85.6679],[0, 0, 1]))
            dist = np.asarray([0.0179, -1.8783,0.0010,-0.0013, 2.7383])
            grid_size = 20
            height, width = cv_image.shape
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width, height), 1, (width, height))
            dst = cv2.undistort(cv_image, mtx, dist, None, newcameramtx)
            x, y, w, h = roi
            dst = dst[y:y + h, x:x + w]


            #cv2.line(dst, (width/2, 0), (width/2, height), (255, 0, 0), 1, 8)
            #cv2.line(dst, (0, height/2), (width, height/2), (255, 0, 0), 1, 8)

            #for x in range(0, width/2 -1, grid_size):
            #    cv2.line(cv_image,(a+x, 0), (a+x, height), (255,0, 0), 1, 8)
            #    cv2.line(cv_image,(a-x, 0), (a-x, height), (255,0, 0), 1, 8)

            #cv2.namedWindow('Image amplitude undis', cv2.WINDOW_NORMAL)
            #cv2.imshow("Image amplitude undis", dst)
            #cv2.resizeWindow('Image amplitude undis', 600, 600)
            cv2.namedWindow('Image amplitude', cv2.WINDOW_NORMAL)
            cv2.imshow("Image amplitude", cv_image)
            cv2.resizeWindow('Image amplitude', 600, 600)
            cv2.imwrite("/home/jiongrui/Matlab/chess_1.jpg", cv_image)
            #cv2.imwrite("/catkin_ws/src/Images/boxamp.jpg", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)


def main(args):
    ic = ImageConverter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
