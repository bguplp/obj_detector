#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from obj_detector.srv import individual, individualResponse
from sensor_msgs.msg import CompressedImage
import face_recognition
import os
import glob
import string

# from cv_bridge import CvBridge, CvBridgeError

# This code is heavily based on https://github.com/ageitgey/face_recognition.git
# The integration to ROS was made by Orel Hamamy, this script create service for
# recognize person the current frame from 'camera_topic'. The service call request
# is a name of person, if the individual is in the frame the service responde with 
# the face center location.

camera_topic = '/kinect2/qhd/image_color_rect/compressed'

def recognotion_cb(msg):
    res = individualResponse()
    res.res = False
    msg.name = ''.join(filter(lambda c: c in string.printable, msg.name))

    img = rospy.wait_for_message(camera_topic, CompressedImage)
    img = np.fromstring(img.data, np.uint8)
    img = cv2.imdecode(img, cv2.IMREAD_COLOR)
    img = img[:, :, ::-1]

    face_locations = face_recognition.face_locations(img)
    face_encodings = face_recognition.face_encodings(img, face_locations)
    face_names = []

    for face_location, face_encoding in zip(face_locations, face_encodings):
        # See if the face is a match for the known face(s)
        matches = face_recognition.compare_faces(faces_encodings, face_encoding)
        name = "Unknown"
        # # If a match was found in known_face_encodings, just use the first one.
        # if True in matches:
        #     first_match_index = matches.index(True)
        #     name = faces_names[first_match_index]

        # Or instead, use the known face with the smallest distance to the new face
        face_distances = face_recognition.face_distance(faces_encodings, face_encoding)
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = faces_names[best_match_index]
        if name == msg.name:
            res.res = True

            # The structure of face_location is (top, right, bottom, left)
            res.pose.x_center = (face_location[1]+face_location[3])/2
            res.pose.y_center = (face_location[0]+face_location[2])/2
            return res
        face_names.append(name)
    return res


def main():
    global faces_encodings, faces_names
    faces_encodings = []
    faces_names = []

    path = os.path.join(os.path.abspath(__file__ + '/../../'), "faces/")
    list_of_files = [f for f in glob.glob(path+'*.jpg')]
    for i, img in enumerate(list_of_files):
        globals()['image_{}'.format(i)] = face_recognition.load_image_file(img)
        faces_encodings.append(face_recognition.face_encodings(globals()['image_{}'.format(i)])[0])
        faces_names.append(img.replace(path, '').replace('.jpg', ''))
        del globals()['image_{}'.format(i)]
    rospy.Service('face_recognizer', individual, recognotion_cb)

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('face_recognition', anonymous=False)
    main()
