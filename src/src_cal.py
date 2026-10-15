#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
import glob
import os

#termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
points = np.zeros((6*7, 3), np.float32)
points[:,:2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
object_points = []
image_points = []
catkin_ws_path = os.getenv("CMAKE_PREFIX_PATH", default=None)

#Function to fetch all the imgages
def camera_calib():
    #initializing the node
    rospy.init_node('camera_calib')
    img = None
    corners = None
    corners2 = None
    ret = None

    #fetching images
    if catkin_ws_path == None:
        print("Kindly source the package setup.sh file. (devel/setup.sh)")
    image_path = catkin_ws_path.split(":")[0]+"/../src/images/"
    images = glob.glob(image_path+"*.jpg")
    print("PAth is : "+str(image_path+"*.jpg"))
    print("Fetching images")
    print(f"Found %d images"%len(images))
    i = 0
    for imgframe in images:
        print(f"Processing image: %d"%i)
        img = cv.imread(imgframe)
        img = cv.resize(img, (int(img.shape[1]*(0.5)), int(img.shape[0]*(0.6))), interpolation=cv.INTER_AREA)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        #finding the chessboard corners
        ret, corners = cv.findChessboardCorners(gray, (7, 6), None)

        if ret == True:
            print()
            object_points.append(points)
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            image_points.append(corners2)
            #img = cv.resize(img, (int(img.shape[1]*(0.6)), int(img.shape[0]*(0.6))), interpolation=cv.INTER_AREA)
        i+=1
    print("Evaluating camera caliberation")
    ret, mat, dist, rvecs, tvecs = cv.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None)
    print("Camera Intrinsic parameters are:")
    print(mat)

    rospy.spin()


if __name__ == "__main__":
    try:
        camera_calib()
    except rospy.ROSInterruptException:
        pass




