#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import sys
from std_msgs.msg import String, Int16, Header
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from sensor_msgs.msg import CompressedImage, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from obj_detector.msg import Detection_msg
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import tf2_ros
import tf2_geometry_msgs
import copy

# --------------------------description----------------------
# This script getting names of items as args, if it detect  
# one of them its publish PoseStamped and image with the dot in the item center.
# This script get

### inputs argument??? 

pointcloud_topic = '/kinect2/qhd/points'
camera_topic = '/kinect2/qhd/image_color_rect/compressed'
detect_topic = '/yolo4_result/detections'
markers_publish_topic = '/detected_objects'
img_publish_topic = '/yolo4_result/obj_pick/compressed'


class pointcloud():
    def __init__(self):
        #self.data = PointCloud2()
        rospy.Subscriber(pointcloud_topic,PointCloud2,self.listener)

    def listener(self,msg):
        self.data = msg

class Alvar_markers():
    def __init__(self,publisher):
        self.data = AlvarMarkers()
        self.data.markers = [AlvarMarker() , AlvarMarker()]
        self.pub = publisher
        self.data.markers[0].id = 1
        self.data.markers[1].id = 2
        self.data.header.frame_id = 'base_footprint'

    def create(self, location, score):
        global tf_buffer 
        rate = rospy.Rate(5.0)
    
        try:
            transform = tf_buffer.lookup_transform('base_footprint',
                                        location.header.frame_id, #source frame
                                        rospy.Time(0), #get the tf at first available time
                                        rospy.Duration(1.0)) #wait for 1 second
            pose_transformed = tf2_geometry_msgs.do_transform_pose(location, transform)
            self.data.header.stamp = rospy.Time.now()
            
            self.data.markers[0].pose = pose_transformed
            self.data.markers[0].header = location.header
            self.data.markers[0].header.frame_id = self.data.header.frame_id
            self.data.markers[0].confidence = score
            self.data.markers[0].pose.pose.position.z +=0.0
            self.data.markers[0].pose.pose.orientation = Quaternion(0, 0, 0, 1)
                    
            self.data.markers[1].pose = copy.deepcopy(pose_transformed)
            self.data.markers[1].header = location.header
            self.data.markers[1].confidence = score
            self.data.markers[1].pose.pose.position.z -=0.08
            self.data.markers[1].pose.pose.orientation = Quaternion(0, 0, 0, 1)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
    
    def publish(self, event):
        self.pub.publish(self.data)

def img_listner(msg):
    global kinect2_img
    img_cv2 = np.fromstring(msg.data, np.uint8)
    kinect2_img = cv2.imdecode(img_cv2, cv2.IMREAD_COLOR)
        
def createCompresseImage(cv2_img):
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', cv2_img)[1]).tostring()
    return msg

def publish_image(stamp_pose, detection):
    global kinect2_img, pub_img
    frame = kinect2_img
    [x, y] = [int(detection.pose.x_center), int(detection.pose.y_center)]
    cv2.circle(frame, (x,y), radius=3, color=(0, 0, 255), thickness=-5)
    cv2.putText(frame, detection.class_id, (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    compress_image = createCompresseImage(frame)
    pub_img.publish(compress_image)
    
def list_2_stampPose(location):
    global tf_buffer, tf_listener
    pose = Point(location[0], location[1], location[2])
    input_pose = PoseStamped()
    input_pose.pose.position = pose
    input_pose.pose.orientation.w = 1
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'kinect2_depth_optical_frame'
    return input_pose 


def detection_cb(msg, items, point_cloud_data, alvar_marker):
    global kinect2_img, pub_img
    if msg.class_id in items:
        # global pose_pub
        [u ,v] = [int(msg.pose.x_center), int(msg.pose.y_center)]
        try:
            location_xyz = list(pc2.read_points(point_cloud_data.data, ('x', 'y', 'z'), skip_nans = True, uvs=[[u,v]])) # location type --> [(touple)]
            converted_location = list_2_stampPose(location_xyz[0])
            alvar_marker.create(converted_location, msg.score)
            publish_image(converted_location, msg)
            # pose_pub.publish(converted_location)
        except:
            pass
    else:
        compress_image = createCompresseImage(kinect2_img)
        pub_img.publish(compress_image)

        

def main():
    
    point_cloud_data = pointcloud()
    global tf_buffer, tf_listener, pub_img, pub_markers #, pose_pub

    pub_markers = rospy.Publisher(markers_publish_topic, AlvarMarkers, queue_size= 10)
    pub_img = rospy.Publisher(img_publish_topic, CompressedImage, queue_size=  1)
    # pose_pub = rospy.Publisher('/find_objects_node/object_pose', PoseStamped, queue_size= 1)
    alvar_marker = Alvar_markers(pub_markers)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    

    rospy.Subscriber(camera_topic, CompressedImage, img_listner, queue_size=1)
    rospy.wait_for_message(camera_topic, CompressedImage)
    items = rospy.get_param("~items",['cup', 'bottle'])
    detection_cb_lambda = lambda data: detection_cb(data, items, point_cloud_data, alvar_marker)
    
    rospy.Subscriber(detect_topic, Detection_msg, detection_cb_lambda, queue_size= 1)
    rospy.wait_for_message(detect_topic, Detection_msg)
    rospy.Timer(rospy.Duration(0.2), alvar_marker.publish)    
    rospy.spin()

if __name__=='__main__':
    rospy.init_node('alvar_marker_pub', anonymous=True)
    main()