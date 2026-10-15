# Import Monocular Visual Odometry 
import mono_vo as mv

# You may only use the functions imported below
from matplotlib import pyplot as plt
import numpy as np
import cv2
import rospy
import math

MAX_FRAMES = 1000 # Max frames in dataset
MIN_FEATURES = 1500 # Minimum number of features for reliable relative pose between frames

def plotTraj(pose):
    # Write function to plot visual odometry using Matplotlib
    pass

def computeError(pose, truth):
    # Write a function to calculate ATE error between VO trajectory and ground truth
    ATE = 0

    return ATE


### Complete this section by following the instructions below ###

### Task 1
# Get pose from VO
# valid boolean is false if the relative pose is of bad quality, ignore this pose and use the next two frames
# Example call of function 
rotation_matrix, translation_vector, valid = mv.monoVO(frame0, frame1, MIN_FEATURES)
# Hint:  frame0 and frame1 will come from your subscriber so this will be called from your subscriber callback function
# Calculate pose using: relative_pose = rotation_matrix.dot(translation_vector)
# Hint: You might have to convert into a suitable data structure before using pose in task 3 and 4

### Task 2
# Get ground truth
# Extract pose from pose.txt provided in dataset - Columns 3 and 11 correspond to x and z values of pose
# Extract pose from txt file and publish as geometry_msgs/Twist and subscribe to plot and compute error
# Hint: You will add truth poses to a data structure in the subscriber call back and then use this data structure in tasks 3 and 4

### Task 3
# Complete plotTraj function and plot VO and ground truth trajectories
plotTraj(pose)
plotTraj(truth)


### Task 4
ATE = computeError(pose, truth)
print ("Absolute Trajectory Error = " + str(ATE))
