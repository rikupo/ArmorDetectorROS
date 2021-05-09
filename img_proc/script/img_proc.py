#! /usr/bin/python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
import sys
import time
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np

print("Python Version: "  + str(sys.version))

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # Save your OpenCV2 image as a jpeg
    cv2_img = np.array(cv2_img,dtype = np.uint8)
    
    cv2.imshow("Received Image",cv2_img)
    print(cv2_img)
    cv2.imwrite('camera_image.jpeg', cv2_img)
    cv2.waitKey(1)
    # ここでpublishしたい
    imgMsg = bridge.cv2_to_imgmsg(cv2_img, "bgr8")
    pub = rospy.Publisher('detected_image', Image, queue_size=10)
    pub.publish(imgMsg)

def main():
    print("hello ros")
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/img_publisher/image"
    print("waiting for topic")
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

main()