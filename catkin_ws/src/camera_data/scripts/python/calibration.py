import numpy as np
import cv2
import glob
from matplotlib import pyplot as plt

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
print objp

#objp = np.zeros((5*7, 3), np.float32)
#objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

#images = glob.glob('/home/jiongrui/catkin_ws/src/Images/*.png')
#images = glob.glob('/home/jiongrui/catkin_ws/src/Images/chessboard/*.jpg')
images = glob.glob('/home/jiongrui/catkin_ws/src/Images/original/*.png')

#images = glob.glob('/home/jiongrui/catkin_ws/src/Images/edited/*.png')
i=1
for imagesname in images:
    img = cv2.imread(imagesname)
    print imagesname
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #plt.imshow(img,'gray')
    #plt.figure(2)
    #plt.imshow(gray, 'gray')
    #plt.show()
    # Find the chess board corners
    #ret, corners = cv2.findChessboardCorners(gray, (6, 4), None)
    ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

    #print corners

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        #print "corner coordinates:", corners
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7, 6), corners2, ret)
        #img = cv2.drawChessboardCorners(img, (6, 4), corners2, ret)
        #cv2.imwrite('/home/jiongrui/catkin_ws/src/Images/drawCornersE%d.png'%i, img)
        i = i+1
        cv2.imshow('img', img)
        cv2.waitKey(500)
    #print objpoints
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print "ret:", type(ret)
print "mtx:", mtx
print "distCoeff:", dist
print "tvec:", tvecs
#np.savetxt('cameraMatrixO', mtx)
#np.savetxt('distortionO', dist) # np.asarray(rvecs), np.asarray(tvecs))
cv2.destroyAllWindows()
