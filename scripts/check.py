#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int16
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
#from cv_bridge import CvBridge, CvBridgeError

def main():
    pose = PoseStamped()
    pose.pose.position.x = 0.23
    pose.pose.position.z = 0.92
    pose.pose.orientation.w = 1

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform('base_footprint',
                                'kinect2_depth_optical_frame', #source frame
                                rospy.Time(0), #get the tf at first available time
                                rospy.Duration(1.0)) #wait for 1 second
    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
    print(pose_transformed)


if __name__=='__main__':
    rospy.init_node('alva', anonymous=True)
    main()