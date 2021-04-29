#!/usr/bin/env python2.7

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, PointCloud2
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Quaternion
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf

global kinect2


class pointcloud():
    def __init__(self, cloud_topic):
        rospy.Subscriber(cloud_topic, PointCloud2, self.cb)

    def cb(self, msg):
        self.data = msg


def edges_map(heights, th=0.05, display = False):
    heights = np.abs(heights.reshape(heights.shape[0:2]))
    height_sh = heights.shape
    # Calculate the horizinal and vertical edges and sum them
    edges_map = (np.abs(np.concatenate((np.diff(heights, axis=0),
                                    np.zeros((1, height_sh[1]))), axis=0)) +
                np.abs(np.concatenate((np.diff(heights, axis=1),
                np.zeros((height_sh[0], 1))), axis=1)))
    edges_map[edges_map < th] = 0
    edges_map[edges_map >= th] = 1
    edges_map = (edges_map*255).astype(np.uint8)
    if display:
        cv2.imshow('heights2', edges_map)
        cv2.waitKey(1)
    return edges_map

def cb_img(msg):
    global kinect2
    img = np.fromstring(msg.data, np.uint8)
    kinect2 = cv2.imdecode(img, cv2.IMREAD_COLOR)


def main():
    # pc_topic = rospy.get_param('~pointCloudTopic')
    rospy.Subscriber('kinect2/qhd/image_color_rect/compressed',
                     CompressedImage, cb_img)
    pc_topic = '/kinect2/qhd/points'
    pcl = pointcloud(pc_topic)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.wait_for_message(pc_topic, PointCloud2)
    z_vector = PoseStamped()
    z_vector.pose.position.x = 0.0
    z_vector.pose.position.y = 0.
    z_vector.pose.position.z = 0.0
    z_vector.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, -(np.pi/2), 0))
    z_vector.header.frame_id = 'base_link'
    z_vector.header.stamp = rospy.Time(0)
    while True:
        # timing = rospy.Time.now() 
        # Transform to kinect frame
        plane_vector = tf_buffer.transform(z_vector,'kinect2_depth_optical_frame')

        vec = -np.array([plane_vector.pose.position.x, plane_vector.pose.position.y, plane_vector.pose.position.z]).reshape((3, 1))  # Get the normal to the ground (base_link)
        vec = vec/np.linalg.norm(vec)
        point_cloud = np.asarray(list(pc2.read_points(
                                      pcl.data, ('x', 'y', 'z')))).reshape(
                                          (pcl.data.height, pcl.data.width, 3))
        point_cloud = np.nan_to_num(point_cloud)
        calculation = np.matmul(point_cloud, vec)

        if np.min(calculation) <= 0:
            calculation += abs(np.min(calculation))
        # print(np.max(calculation))
        calculation = calculation/np.max(calculation)
        # print(calculation)
        # cv2.imshow('heights', calculation)
        # cv2.waitKey(1)
        edges = edges_map(calculation, display=False)
        cv2.imshow('heights', edges)
        cv2.waitKey(1)
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('table_detection')
    main()


''' # Old try to find surface with line and intersection points
   minLineLength = 300
    maxLineGap = 20
    img_temp = kinect2
    lines = cv2.HoughLinesP(heights_map,1,np.pi/180,50,minLineLength,maxLineGap)[:,0,:]
    # print(lines[0:2])
    x1,y1,x2,y2 =lines[:,0], lines[:,1], lines[:,2],lines[:,3]
    # print(y2-y1)
    # print(x2-x1)
    slopes = np.divide((y2-y1),(x2-x1), dtype = np.float32)
    # print('slope',slopes.shape, (y1-slopes*x1).shape)
    lines_moments = np.concatenate((slopes, y1-slopes*x1),axis = 0).reshape((-1,2),order='F') # Represent all lines properties, y = m*x + c
    # print(lines_moments[0:2])
    intersection_points = []
    for n, line in enumerate(lines_moments):
        # print('line',line)
        for compere_line in lines_moments[n+1:]:
            # print('second line', compere_line)
            if (abs(compere_line[0]-line[0]))<2e-3: # Check if the lines perpendicular, if they are there isn't intersection point
                intersec_x = (compere_line[1]-line[1])/(line[0]-compere_line[0]) # (C_2-C_1)/(m_1-m_2)
                print('X',intersec_x)
                if intersec_x< 0 or intersec_x>height_sh[1] or intersec_x!=intersec_x: 
                    continue
                intersec_y = line[0]*intersec_x + line[1] # Calc the y intersection with putting the x intersection in the line equation
                print('Y',intersec_y)
                if intersec_y< 0 or intersec_y>height_sh[0] or intersec_y!=intersec_y: 
                    continue
                intersection_points.append(np.array((int(intersec_x),int(intersec_y))).reshape((1,2)))
                # print(type(intersection_points))
                # print(len(intersection_points))
    # contours, hierarchy = cv2.findContours(heights_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print (intersection_points)
    
    for x1,y1,x2,y2 in lines:
        cv2.line(img_temp,(x1,y1),(x2,y2),(0,0,255))
    cv2.circle(img_temp,(165,310), 5,(255,0,0),thickness=-1 )
    for point in intersection_points:
        # print (point[0])
        cv2.circle(img_temp,tuple(point[0]), 5,(255,0,0),thickness=-1 )

        # cv2.drawContours(img_temp,contours,-1, (0,0,255) )
    cv2.imshow('heights', img_temp)
    cv2.waitKey(1)
    '''
