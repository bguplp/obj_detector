#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int16
from sensor_msgs.msg import CompressedImage, PointCloud2
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Quaternion
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
import matplotlib.pyplot as plt
import tf

global kinect2


class pointcloud():
    def __init__(self, cloud_topic):
        rospy.Subscriber(cloud_topic, PointCloud2, self.cb)

    def cb(self, msg):
        self.data = msg


def find_surfes(heights, th = 0.25):
    heights = np.abs(heights.reshape(heights.shape[0:2]))
    height_sh = heights.shape
    heights_map = np.abs(np.concatenate((np.diff(heights, axis = 0),np.zeros((1,height_sh[1]))),axis= 0)) + np.abs(np.concatenate((np.diff(heights, axis = 1),np.zeros((height_sh[0],1))),axis= 1))   # Calculate the horizinal edge
    heights_map = (heights_map*255).astype('uint8')
    cv2.imshow('heights2', heights_map)
    minLineLength = 100
    maxLineGap = 50
    img_temp = kinect2
    lines = cv2.HoughLinesP(heights_map,1,np.pi/180,200,minLineLength,maxLineGap)
    print(lines)
    # print(type(kinect2))
    for ii in xrange(len(lines)):
        for x1,y1,x2,y2 in lines[ii]:
            cv2.line(img_temp,(x1,y1),(x2,y2),(0,255,0),2)

    cv2.imshow('heights', img_temp)
    cv2.waitKey(1)


def cb_img(msg):
    global kinect2
    img = np.fromstring(msg.data, np.uint8)
    kinect2 = cv2.imdecode(img,cv2.IMREAD_COLOR)
    


def main():
    # pc_topic = rospy.get_param('~pointCloudTopic')
    rospy.Subscriber('kinect2/qhd/image_color_rect/compressed', CompressedImage, cb_img)
    pc_topic = '/kinect2/qhd/points'
    pcl = pointcloud(pc_topic)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)    
    rospy.wait_for_message(pc_topic, PointCloud2)
    z_vector = PoseStamped()
    z_vector.pose.position.x = 0.0
    z_vector.pose.position.y = 0.
    z_vector.pose.position.z = 0.0
    z_vector.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,-(np.pi/2),0))
    z_vector.header.frame_id = 'base_link'
    z_vector.header.stamp = rospy.Time(0)
    while True:
            # timing = rospy.Time.now() 
            plane_vector = tf_buffer.transform(z_vector, 'kinect2_depth_optical_frame') # Transform to kinect frame
            vec = -np.array([plane_vector.pose.position.x, plane_vector.pose.position.y, plane_vector.pose.position.z]).reshape((3,1)) # Get the normal to the ground (base_link)
            vec = vec/np.linalg.norm(vec)
            point_cloud = np.asarray(list(pc2.read_points(pcl.data, ('x','y','z')))).reshape((pcl.data.height,pcl.data.width,3))
            point_cloud = np.nan_to_num(point_cloud)
            calculation = np.matmul(point_cloud,vec)
            
            if np.min(calculation)<=0:
                calculation += abs(np.min(calculation))
            # print(np.max(calculation))
            calculation = calculation/np.max(calculation)
            # print(calculation)
            # cv2.imshow('heights', calculation)
            # cv2.waitKey(1)
            find_surfes(calculation)
            
            
            rospy.sleep(1)

    
    

if __name__=='__main__':
    rospy.init_node('table_detection')
    main()