#!/usr/bin/env python

import sys
import time
#from tkinter import TRUE
import roslib
from math import pi
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
#from dh_gripper_msgs.msg import GripperCtrl
from moveit_commander.conversions import pose_to_list
#from scripts.arjun2 import BASE_ADJ
import tf2_msgs.msg
import tf
import numpy as np
#from mask import ImageProcessing
import cv2
#import imutils
#from darknet_ros_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import random

#goalList=[[-185 , 36 , 54, 172 , -91 , -181],[-145 , 42 , 54 , 172 , -90 , -181],[-145,48, 54 ,167 ,-90 ,-181]]

LABELS = ["Yellow", "Cube", "Arc"]

pick_down = [-185 , 36 , 54, 172 , -91 , -181]
WIDTH_THRESH = [140, 200,220 ]

INPUT_WIDTH = 416
INPUT_HEIGHT = 416
SCORE_THRESHOLD = 0.2
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.4

colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

is_cuda = len(sys.argv) > 1 and sys.argv[1] == "cuda"

#net = build_model(is_cuda)
#capture = load_capture()

INPUT_FILE='14.jpg'
OUTPUT_FILE='predicted.jpg'
LABELS_FILE='classes.names'
CONFIG_FILE='yolov3-custom.cfg'
WEIGHTS_FILE='yolov3-custom_6000_300.weights'
CONFIDENCE_THRESHOLD=0.3

#is_cuda = len(sys.argv) > 1 and sys.argv[1] == "cuda"
cap = cv2.VideoCapture(-1)

rospy.init_node("camera_pub", anonymous = True)

def publish():
    bridge = CvBridge()
    pub = rospy.Publisher("/yolo_image", Image, queue_size=1)
    #rate = rospy.Rate(0.2)
    while True :
        ret, frame = cap.read()

        if ret :
            image = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(image)
            print("Published")
            cv2.imshow("image", frame)
            if cv2.waitKey(1) & 0xFF == ord('q') :
                break
            #rate.sleep()

try :
    publish()
except :
    print("error")
    cv2.destroyAllWindows()