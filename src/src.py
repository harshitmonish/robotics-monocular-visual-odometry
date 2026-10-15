#!/usr/bin/env python
# Import Monocular Visual Odometry 
import mono_vo as mv

# You may only use the functions imported below
from matplotlib import pyplot as plt
import numpy as np
import cv2
import rospy
import math
from collections import defaultdict
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv


#global variables
MAX_FRAMES = 1000 # Max frames in dataset
MIN_FEATURES = 1500 # Minimum number of features for reliable relative pose between frames
ground_truth = defaultdict(list)
pose_estimation = defaultdict(list)
prev_img = None
prev_img_id = None
prev_rot_mat = []
prev_tran_vec = []
publisher_pose_gt = rospy.Publisher("/Kitti_ground_Truth_marker", Marker, queue_size=10)
publisher_pose_traj = rospy.Publisher("/kitti_Trajectory_marker", Marker, queue_size=10)
bridge = CvBridge()


def plotTraj(pose):
    # Write function to plot visual odometry using Matplotlib
    pass

def computeError(pose, truth):
    # Write a function to calculate ATE error between VO trajectory and ground truth
    ATE = 0

    return ATE

def callback_pose(msg):
    if msg.header.seq <= 1:
        global prev_tran_vec
        prev_tran_vec = np.array([msg.twist.linear.x, msg.twist.linear.y, 0])
        return 
    global ground_truth
    ground_truth[msg.header.seq] = [msg.twist.linear.x, msg.twist.linear.y]

    marker2 = Marker()
    marker2.header.frame_id = "map"
    marker2.header.stamp = rospy.Time.now()
    marker2.type = Marker.POINTS
    marker2.action = Marker.ADD
    marker2.scale.x = 0.5
    marker2.scale.y = 0.5
    marker2.color.g = 0.0
    marker2.color.r = 1.0
    marker2.color.b = 0.0
    marker2.color.a = 1.0
    marker2.id = msg.header.seq
    pos = Point()
    pos.x = float(msg.twist.linear.x)
    pos.y = float(msg.twist.linear.y)
    marker2.points.append(pos)
    publisher_pose_gt.publish(marker2)
    #prev_pos = pos

def callback_img(img):
    global pose_estimation
    global prev_img
    global prev_img_id
    global prev_tran_vec
    global prev_rot_mat


    frame_id = img.header.seq
    img = bridge.imgmsg_to_cv2(img,"bgr8")
    #img = np.asarray(img.data)
    if frame_id <= 1:
        prev_img = img
        prev_img_id = frame_id
        return 
    rotation_matrix, translation_vector, valid = mv.monoVO(img, prev_img, MIN_FEATURES)
    prev_img = img
    prev_img_id = frame_id
    if valid:
        if len(prev_rot_mat) != 0:
            new_trans_vec = prev_tran_vec + prev_rot_mat.dot(translation_vector)
            new_rot_mat = prev_rot_mat.dot(rotation_matrix)

            #relative_pos = rotation_matrix.dot(translation_vector)
            pose_estimation[frame_id] = [new_trans_vec[0], new_trans_vec[1]]
            prev_tran_vec = new_trans_vec
            prev_rot_mat = new_rot_mat


            marker1 = Marker()
            marker1.header.frame_id = "map"
            marker1.header.stamp = rospy.Time.now()
            marker1.type = Marker.POINTS
            marker1.action = Marker.ADD
            marker1.scale.x = 0.5
            marker1.scale.y = 0.5
            marker1.color.g = 0.0
            marker1.color.r = 0.0
            marker1.color.b = 1.0
            marker1.color.a = 1.0
            marker1.id = frame_id
            pos = Point()
            pos.x = float(new_trans_vec[0])
            pos.y = float(-new_trans_vec[2])
            marker1.points.append(pos)
            publisher_pose_traj.publish(marker1)
        else:
            prev_tran_vec = translation_vector
            prev_rot_mat = rotation_matrix

    if frame_id == 999:
        evaluate_error()


def evaluate_error():
    print("Evaluating Absolute Trajectory Error!")
    error = 0
    n = len(ground_truth)
    for key, val in ground_truth.items():
        if key not in pose_estimation:
            continue
        x1, y1 = val[0], val[1]
        x2, y2 = pose_estimation[key][0], pose_estimation[key][1]
        error += ((x2 - x1) + (y2 - y1 ))**2
    ate = math.sqrt(error//n)
    print("Absolute Trajectory Error = "+str(ate))


def main():
    rospy.init_node('lab4_vo')
    frame_rate = rospy.get_param("/lab4/fps")
    r = rospy.Rate(frame_rate)

    img_sub_node = rospy.Subscriber('/kitti_image', Image, callback_img, queue_size=100)
    ground_truth_sub_node =  rospy.Subscriber('/kitti_pose', TwistStamped, callback_pose)
    r.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
