#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int16
from sensor_msgs.msg import CompressedImage, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
import matplotlib.pyplot as plt

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
    while True: 
            trans = tf_buffer.lookup_transform('base_footprint', # target frame
                                                pcl.data.header.frame_id, # source frame
                                                rospy.Time(0), # time
                                                rospy.Duration(1)) # 
            cloud_out = do_transform_cloud(pcl.data, trans)
            a = list(pc2.read_points(cloud_out, ('x','y','z')))
            b = np.asarray(a).reshape((pcl.data.height,pcl.data.width,3))
            print(np.nanmax(b[:,:,0]))
            b = cv2.normalize(b,None , alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype = cv2.CV_32F)
            cv2.imshow('x', b[:,:,0])
            cv2.imshow('y', b[:,:,1])
            cv2.imshow('z', b[:,:,2])
            print(np.nanmax(b[:,:,0]))
            if cv2.waitKey(1)==27:
                break
            rospy.sleep(rospy.Duration(1))

    
    

if __name__=='__main__':
    rospy.init_node('table_detection')
    main()