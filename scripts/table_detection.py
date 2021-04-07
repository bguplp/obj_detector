#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int16
from sensor_msgs.msg import CompressedImage, PointCloud2
from tf2_geometry_msgs import PoseStamped
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
import matplotlib.pyplot as plt
import tf

class pointcloud():
    def __init__(self, cloud_topic):
        rospy.Subscriber(cloud_topic, PointCloud2, self.cb)
    
    def cb(self, msg):
        self.data = msg

def main():
    # pc_topic = rospy.get_param('~pointCloudTopic')
    pc_topic = '/kinect2/qhd/points'
    pcl = pointcloud(pc_topic)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)    
    rospy.wait_for_message(pc_topic, PointCloud2)
    z_vector = PoseStamped()
    z_vector.pose.position.x = 0.0
    z_vector.pose.position.y = 0.
    z_vector.pose.position.z = 1.0
    z_vector.pose.orientation.w = 0.
    z_vector.header.frame_id = 'map'
    z_vector.header.stamp = rospy.Time(0)
    # print(z_vector)

    while True:
            # timing = rospy.Time.now() 
            plane_vector = tf_buffer.transform(z_vector, 'kinect2_depth_optical_frame')
            print(plane_vector)
            quat = [plane_vector.pose.orientation.x, plane_vector.pose.orientation.y, plane_vector.pose.orientation.z, plane_vector.pose.orientation.w]
            vec = tf.transformations.euler_from_quaternion(quat)
            print('euler' + str(vec))
            '''
            quat = [plane_vector.pose.orientation.x, plane_vector.pose.orientation.y, plane_vector.pose.orientation.z, plane_vector.pose.orientation.w]
            vec = tf.transformations.euler_from_quaternion(quat)
            vec = np.array([plane_vector.x, plane_vector.y, plane_vector.z]).reshape((3,1))
            vec = vec/np.linalg.norm(vec)
            
            plane_vector = plane_vector.pose.position
            plane_vector = -np.array([plane_vector.x, plane_vector.y, plane_vector.z])
            '''
            yaw = vec[2] ; pitch = vec[1]; roll = vec[0]
            x = -np.cos(yaw)*np.sin(pitch)*np.sin(roll)-np.sin(yaw)*np.cos(roll)
            y = -np.sin(yaw)*np.sin(pitch)*np.sin(roll)+np.cos(yaw)*np.cos(roll)
            z =  np.cos(pitch)*np.sin(roll)
            vec = np.array([x, y, z]).reshape((3,1))
            vec = vec/np.linalg.norm(vec)
            print('dir:' + str(vec))
            point_cloud = np.asarray(list(pc2.read_points(pcl.data, ('x','y','z')))).reshape((pcl.data.height,pcl.data.width,3))
            point_cloud = np.nan_to_num(point_cloud)
            calculation = np.matmul(point_cloud,vec)
            
            
            if np.min(calculation)<=0:
                calculation += abs(np.min(calculation))
            # print(np.max(calculation))
            calculation = calculation/np.max(calculation)
            # print(calculation)
            cv2.imshow('cal', calculation)
            cv2.waitKey(1)
            
            
            rospy.sleep(1)

    
    

if __name__=='__main__':
    rospy.init_node('table_detection')
    main()