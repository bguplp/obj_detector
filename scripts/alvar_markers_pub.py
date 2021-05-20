#!/usr/bin/env python

import rospy
import cv2
import numpy as np
# from std_msgs.msg import String, Int16, Header
from obj_detector.srv import items, itemsResponse
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from sensor_msgs.msg import CompressedImage, PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from obj_detector.msg import Detection_msg
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import tf2_ros
import tf2_geometry_msgs
import copy
import string

# --------------------------description---------------------- /
# This script is a serviec that gets names of items, then if it detect
# one of them its publish PoseStamped and image with the dot in the item center.
# The PoseStamped are publish at 2 different topic according to the detect object.
# One topic for pick (cup, bottle) second topic for cup deliver to person.
# For intput '' the servie stop, for input like 'cup,bottle' the service publish
# alvar marker of cup or bottle in the pick up matker topic.
# Additionaly this script include KLF that estimate the object location


pointcloud_topic = '/kinect2/qhd/points'
camera_topic = '/kinect2/qhd/image_color_rect/compressed'
detect_topic = '/yolo4_result/detections'
pick_markers_publish_topic = '/detected_objects'
person_markers_publish_topic = "/detected_objects_person"
img_publish_topic = '/yolo4_result/obj_pick/compressed'


class pointcloud():
    def __init__(self):
        # self.data = PointCloud2()
        rospy.Subscriber(pointcloud_topic, PointCloud2, self.listener)

    def listener(self, msg):
        self.data = msg


class KLF_alvarMarker():
    def __init__(self):
        self.odom = rospy.wait_for_message('/mobile_base_controller/odom', Odometry).twist.twist
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_cb, queue_size=1)
        self.dt = 0.01
        rospy.Timer(rospy.Duration(self.dt), self.predict)
        self.x = 0.
        self.y = 0.
        self.x_dot = 0
        self.y_dot = 0
        self.x_vec = np.array([[self.x], [self.y]], dtype=np.float32)  # , self.x_dot, self.y_dot)
        self.p = np.zeros((2, 2))
        self.q = np.eye((2))*0.1
        self.r = np.eye((2))*0.01

    def odom_cb(self, msg):
        self.odom = msg.twist.twist

    def predict(self, event):
        temp = self.x_vec[0] if self.x_vec[0] != 0 else 10e5
        F = np.array([[1-self.dt*self.odom.linear.x/temp, 0],
                      [-self.dt*self.odom.angular.z, 1]], dtype=np.float32)

        self.x_vec = np.matmul(F, self.x_vec)  # Predicted state estimate
        self.p = np.linalg.multi_dot([F, self.p, F.T]) + self.q  # Predicted estimate covariance
        # print(self.x_vec)

    def update(self, observe):
        observe_vec = np.array([[observe.pose.position.x], [observe.pose.position.y]])
        y_vec = observe_vec - self.x_vec
        s = self.p + self.r
        k = self.p.dot(np.linalg.inv(s))
        self.x_vec = self.x_vec + k.dot(y_vec)
        self.p = (np.eye(2)-k).dot(self.p)
        self.p = np.linalg.multi_dot([(np.eye(2)-k), self.p, (np.eye(2)-k).T]) + np.linalg.multi_dot([k, self.r, k.T])


class Alvar_markers():
    def __init__(self, publisher):
        self.data = AlvarMarkers()
        self.data.markers = [AlvarMarker(), AlvarMarker()]
        self.pub = publisher
        self.data.markers[0].id = 1

        self.data.markers[1].id = 2
        self.data.header.frame_id = 'base_footprint'
        self.last_update = rospy.Time.now()
        self._timer = rospy.Timer(rospy.Duration(0.2), self.publish)

    def update(self, pose_transformed, score):
        self.data.header.stamp = rospy.Time.now()
        self.last_update = rospy.Time.now()
        self.data.markers[0].pose = pose_transformed
        self.data.markers[0].header.stamp = rospy.Time.now()
        self.data.markers[0].header.frame_id = self.data.header.frame_id
        self.data.markers[0].confidence = score
        self.data.markers[0].pose.pose.position.z += 0.0
        self.data.markers[0].pose.pose.orientation = Quaternion(0, 0, 0, 1)

        self.data.markers[1].pose = copy.deepcopy(pose_transformed)
        self.data.markers[1].header.stamp = rospy.Time.now()
        self.data.markers[1].header.frame_id = self.data.header.frame_id
        self.data.markers[1].confidence = score
        self.data.markers[1].pose.pose.position.z -= 0.08
        self.data.markers[1].pose.pose.orientation = Quaternion(0, 0, 0, 1)

    def predict(self):
        global estimator, pub_img, kinect2_img
        predict_pose = PoseStamped()
        predict_pose.pose.position.x = estimator.x_vec[0]
        predict_pose.pose.position.y = estimator.x_vec[1]
        predict_pose.header.frame_id = 'base_footprint'
        predict_pose.pose.position.z = self.data.markers[0].pose.pose.position.z
        self.update(predict_pose, 0.1)
        compress_image = createCompresseImage(kinect2_img)
        pub_img.publish(compress_image)

    def publish(self, event):
        if ((rospy.Time.now() - self.last_update).to_sec > 0.5):
            self.predict()
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
    cv2.circle(frame, (x, y), radius=3, color=(0, 0, 255), thickness=-5)
    cv2.putText(frame, detection.class_id, (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    compress_image = createCompresseImage(frame)
    pub_img.publish(compress_image)


def list_2_stampPose(location):
    pose = Point(location[0], location[1], location[2])
    input_pose = PoseStamped()
    input_pose.pose.position = pose
    input_pose.pose.orientation.w = 1
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'kinect2_depth_optical_frame'
    return input_pose


def detection_cb(msg, items_list, point_cloud_data):
    global kinect2_img, tf_buffer, estimator
    if not items_list:
        pass
    elif msg.class_id in items_list:
        [u, v] = [int(msg.pose.x_center), int(msg.pose.y_center)]
        # location type --> [(touple)]
        location_xyz = list(pc2.read_points(point_cloud_data.data, ('x', 'y', 'z'), skip_nans=True, uvs=[[u, v]]))
        if location_xyz:
            converted_location = list_2_stampPose(location_xyz[0])
            transform = tf_buffer.lookup_transform('base_footprint',
                                                   converted_location.header.frame_id,  # source frame
                                                   rospy.Time(0),  # get the tf at first available time
                                                   rospy.Duration(1.0))  # wait for 1 second
            pose_transformed = tf2_geometry_msgs.do_transform_pose(converted_location,
                                                                   transform)
            alvar_marker.update(pose_transformed, msg.score)
            estimator.update(pose_transformed)
            publish_image(converted_location, msg)


def item_cb(msg):
    global items_list, alvar_marker, estimator
    msg.items = ''.join(filter(lambda c: c in string.printable, msg.items))
    items_list = msg.items.split(",")
    if '' in items_list:
        # alvar_marker.pub.unregister()
        # rospy.time.Timer.shotdown()
        if alvar_marker:
            alvar_marker._timer._shutdown = True
        del alvar_marker
        del estimator
        alvar_marker = False
        estimator = False
        return itemsResponse(False)
    estimator = KLF_alvarMarker()
    if 'person' in items_list:
        alvar_marker = Alvar_markers(person_pub_markers)
    else:
        alvar_marker = Alvar_markers(pick_pub_markers)
    return itemsResponse(True)

def main():
    point_cloud_data = pointcloud()
    global tf_buffer, tf_listener, pub_img, pick_pub_markers, person_pub_markers, estimator, items_list, alvar_marker
    items_list = []
    alvar_marker = False
    estimator = False
    rospy.Service('alvar_marker_service', items, item_cb)
    pick_pub_markers = rospy.Publisher(pick_markers_publish_topic, AlvarMarkers,
                                       queue_size=10)
    person_pub_markers = rospy.Publisher(person_markers_publish_topic, AlvarMarkers,
                                         queue_size=10)
    pub_img = rospy.Publisher(img_publish_topic, CompressedImage, queue_size=1)
    # pose_pub = rospy.Publisher('/find_objects_node/object_pose', PoseStamped, queue_size= 1)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber(camera_topic, CompressedImage, img_listner, queue_size=1)
    rospy.wait_for_message(camera_topic, CompressedImage)
    detection_cb_lambda = lambda data: detection_cb(data, items_list, point_cloud_data)

    rospy.Subscriber(detect_topic, Detection_msg, detection_cb_lambda, queue_size=15)
    rospy.wait_for_message(detect_topic, Detection_msg)

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('alvar_marker_pub', anonymous=True)
    main()
