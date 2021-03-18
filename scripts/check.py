#!/usr/bin/env python

import rospy
import os
import cv2
import numpy as np
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Image, CompressedImage
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError


abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

class image_convertor():
    def __init__(self,camera_topic,frame_rate):
        self.bridge = CvBridge()
        self.shape = (128,128)
        self.count = 0
        self.date = datetime.now().strftime("%d-%m-%y,%H:%M")
        rospy.Subscriber(camera_topic, Image, self.cb)


    def cb(self, msg):
        self.cv_img = self.bridge.imgmsg_to_cv2(msg,desired_encoding = 'passthrough')
        self.resize_img  = cv2.resize(self.cv_img, self.shape,
                               interpolation=cv2.INTER_LINEAR)

        self.resize_img = self.resize_img/10

        cv2.imshow('img', self.resize_img)
        cv2.waitKey(2)

    def save_img(self, event):
        self.count +=1
        cv2.imwrite('img/{}--{}.jpg'.format(self.date, self.count), self.resize_img)

def main(frame_rate,camera_topic):
    
    image = image_convertor(camera_topic,frame_rate)
    rospy.wait_for_message(camera_topic,Image)
    rospy.Timer(rospy.Duration(1.0/float(frame_rate)), image.save_img)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__=='__main__':
    rospy.init_node('video', anonymous=True)
    frame_rate = rospy.get_param('frame_rate', default=15)
    camera_topic = rospy.get_param('camera_topic', default= '/kinect2/qhd/image_depth_rect')
    main(frame_rate,camera_topic)

'''    #rospy.wait_for_message(camera_topic, Image)
    while not rospy.is_shutdown():
        #print((image.resize_img))
        # out.write(np.uint8(image.resize_img))
        #print(image.shape)
        print()
        '''