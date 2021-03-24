#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int16
from sensor_msgs.msg import CompressedImage, PointCloud2
import sensor_msgs.point_cloud2 as pc2

class pointcloud():
    def __init__(self, cloud_topic):
        rospy.Subscriber(cloud_topic, PointCloud2, self.cb)
    
    def cb(self, msg):
        self.data = msg

def main():
    # pc_topic = rospy.get_param('~pointCloudTopic')
    pc_topic = '/kinect2/qhd/points'
    pcl = pointcloud(pc_topic)
    rospy.wait_for_message(pc_topic, PointCloud2)
    
    while not rospy.is_shutdown():
        a = list(pc2.read_points(pcl.data, ('z')))
        b = np.asarray(a).reshape((pcl.data.height,pcl.data.width))
        print(np.nanmax(b))
        print(np.nanmin(b))
        rospy.sleep(rospy.Duration(1))
    
    

if __name__=='__main__':
    rospy.init_node('table_detection')
    main()