#!/usr/bin/env python
import rospy
import os
import glob
import random
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import cv2 as cv

catkin_ws_path = os.getenv("CMAKE_PREFIX_PATH", default=None)
publisher_pose_gt = rospy.Publisher("/kitti_pose_marker", Marker, queue_size=10)

def get_image_pose():
    if catkin_ws_path == None:
        print("Kindly source the package setup.sh file. (devel/setup.sh)")
    image_path = catkin_ws_path.split(":")[0]+"/../src/dataset/sequence/image_0/"
    pose_data_path = catkin_ws_path.split(":")[0]+"/../src/dataset/pose/00.txt"
    images = sorted(glob.glob(image_path+"*.png"))

    #fetching pose data
    with open(pose_data_path) as f:
        pose_data = f.readlines()
    print(f"Found %d images"%len(images))
    return (images, pose_data)

#callback function to post marker data
def callback(msg):
    marker1 = Marker()
    marker1.header.frame_id = "map"
    marker1.header.stamp = rospy.Time.now()
    marker1.type = Marker.POINTS
    marker1.action = Marker.ADD
    marker1.scale.x = 1.0
    marker1.scale.y = 1.0
    marker1.color.g = 0.0
    marker1.color.r = 1.0
    marker1.color.b = 0.0
    marker1.color.a = 1.0
    marker1.id = msg.header.seq
    pos = Point()
    pos.x = float(msg.twist.linear.x)
    pos.y = float(msg.twist.linear.y)
    marker1.points.append(pos)
    publisher_pose_gt.publish(marker1)


#main function
def main():
    rospy.init_node('view_kitti')
    frame_rate = rospy.get_param("/fps")
    r = rospy.Rate(frame_rate)

    subs_node = rospy.Subscriber('/kitti_pose', TwistStamped, callback)

    pub_node_pose = rospy.Publisher("/kitti_pose", TwistStamped, queue_size=10)
    pub_node_image = rospy.Publisher("/kitti_image", Image, queue_size=10)

    images, pose_data = get_image_pose()
    seq_id = 0

    bridge = CvBridge()

    while seq_id <= 1000:
        #getting groundtruth data
        move = TwistStamped()
        move.twist.linear.x = float(pose_data[seq_id].split(" ")[3])
        move.twist.linear.y = float(pose_data[seq_id].split(" ")[11])
        move.twist.linear.z = 0.0
        move.header.seq = seq_id

        #fetching the image
        img =  cv.imread(images[seq_id])
        msg = bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.seq=seq_id
        pub_node_image.publish(msg)

        #publishing ground truth
        pub_node_pose.publish(move)
        seq_id+=1
        r.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

