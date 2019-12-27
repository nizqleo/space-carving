#!/usr/bin/env python


import sys
import copy
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from moveit_commander.conversions import pose_to_list
from tf import transformations
from cv_bridge import CvBridge, CvBridgeError
import cv2


bridge = CvBridge()
rgb_flag = False
depth_flag = False
stamp = 0

def rgb_image_callback(msg): 
    global rgb_flag

    if rgb_flag == True:   
        rgb_flag = False
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            result=cv2.imwrite('data/rgb/'+ str(stamp) +'.png', cv2_img)
            print(result, "saving rgb image!")

def depth_image_callback(msg): 
    global depth_flag
    if depth_flag == True:  
        depth_flag = False
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "32FC1")
            cv2.imshow("depth", cv2_img)
            cv2.waitKey(0)
        except CvBridgeError, e:
            print(e)
        else:
            result=cv2.imwrite('data/depth/'+ str(stamp) + '.bmp', cv2_img)
            print(result, "saving depth image!")
            
            


def image_saver_callback(msg):
    global rgb_flag
    global depth_flag
    global stamp
    rgb_flag = True
    depth_flag=True
    stamp = msg.data



if __name__ == '__main__':
    rospy.init_node('image_saver', anonymous=True)

    rospy.Subscriber("/camera/rgb/image_raw", Image, rgb_image_callback)
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_image_callback) 
    rospy.Subscriber("/imageSerialNum", Int8, image_saver_callback)
    rospy.spin()