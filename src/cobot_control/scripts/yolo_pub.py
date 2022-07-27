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

from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from cobot_control.msg import boundingboxes
#from darknet_ros_msgs.msg import BoundingBoxes

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

cap = cv2.VideoCapture(-1)


class DNN :
    def __init__(self) :
        #print("init")
        self.H, self.W = 0, 0
        self.result = []
        self.output = []
        
        #self.sub = rospy.Subscriber("yolo_image", Image, self.callback)
        self.pub = rospy.Publisher("bounding_box", boundingboxes, queue_size = 1)
        #self.vel = Twist()
        #self.bridge = CvBridge()
        self.bb = boundingboxes()
        #self.callback(frame)

    def publish(self, frame) :
        #frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.result, image = self.predict(frame)

        self.bb.x1 = 0.0
        self.bb.y1 = 0.0
        self.bb.w1 = 0.0
        self.bb.h1 = 0.0
        self.bb.l1 = 0.0
        self.bb.x2 = 0.0
        self.bb.y2 = 0.0
        self.bb.w2 = 0.0
        self.bb.h2 = 0.0
        self.bb.l2 = 0.0
        self.bb.x3 = 0.0
        self.bb.y3 = 0.0
        self.bb.w3 = 0.0
        self.bb.h3 = 0.0
        self.bb.l3 = 0.0
        
        if len(self.result)==0 :

            self.bb.x1 = 0.0
            self.bb.y1 = 0.0
            self.bb.w1 = 0.0
            self.bb.h1 = 0.0
            self.bb.l1 = 0.0

            self.bb.x2 = 0.0
            self.bb.y2 = 0.0
            self.bb.w2 = 0.0
            self.bb.h2 = 0.0
            self.bb.l2 = 0.0

            self.bb.x3 = 0.0
            self.bb.y3 = 0.0
            self.bb.w3 = 0.0
            self.bb.h3 = 0.0
            self.bb.l3 = 0.0

        elif len(self.result)==1:
            if self.result[0][4] == 0:

                self.bb.x1 = self.result[0][0]
                self.bb.y1 = self.result[0][1]
                self.bb.w1 = self.result[0][2]
                self.bb.h1 = self.result[0][3]
                self.bb.l1 = self.result[0][4]

            elif self.result[0][4] == 1:
                self.bb.x2 = self.result[0][0]
                self.bb.y2 = self.result[0][1]
                self.bb.w2 = self.result[0][2]
                self.bb.h2 = self.result[0][3]
                self.bb.l2 = self.result[0][4]

            elif self.result[0][4] == 2:
                self.bb.x3 = self.result[0][0]
                self.bb.y3 = self.result[0][1]
                self.bb.w3 = self.result[0][2]
                self.bb.h3 = self.result[0][3]
                self.bb.l3 = self.result[0][4]
            
            else:
                pass


            

        elif len(self.result)==2:

            if self.result[0][4] == 0:

                self.bb.x1 = self.result[0][0]
                self.bb.y1 = self.result[0][1]
                self.bb.w1 = self.result[0][2]
                self.bb.h1 = self.result[0][3]
                self.bb.l1 = self.result[0][4]

            elif self.result[0][4] == 1:
                self.bb.x2 = self.result[0][0]
                self.bb.y2 = self.result[0][1]
                self.bb.w2 = self.result[0][2]
                self.bb.h2 = self.result[0][3]
                self.bb.l2 = self.result[0][4]

            elif self.result[0][4] == 2:
                self.bb.x3 = self.result[0][0]
                self.bb.y3 = self.result[0][1]
                self.bb.w3 = self.result[0][2]
                self.bb.h3 = self.result[0][3]
                self.bb.l3 = self.result[0][4]

            else:
                pass

            if self.result[1][4] == 0:

                self.bb.x1 = self.result[1][0]
                self.bb.y1 = self.result[1][1]
                self.bb.w1 = self.result[1][2]
                self.bb.h1 = self.result[1][3]
                self.bb.l1 = self.result[1][4]

            elif self.result[1][4] == 1:
                self.bb.x2 = self.result[1][0]
                self.bb.y2 = self.result[1][1]
                self.bb.w2 = self.result[1][2]
                self.bb.h2 = self.result[1][3]
                self.bb.l2 = self.result[1][4]

            elif self.result[1][4] == 2:
                self.bb.x3 = self.result[1][0]
                self.bb.y3 = self.result[1][1]
                self.bb.w3 = self.result[1][2]
                self.bb.h3 = self.result[1][3]
                self.bb.l3 = self.result[1][4]

            else :
                pass

        else:

            if self.result[0][4] == 0:

                self.bb.x1 = self.result[0][0]
                self.bb.y1 = self.result[0][1]
                self.bb.w1 = self.result[0][2]
                self.bb.h1 = self.result[0][3]
                self.bb.l1 = self.result[0][4]

            elif self.result[0][4] == 1:
                self.bb.x2 = self.result[0][0]
                self.bb.y2 = self.result[0][1]
                self.bb.w2 = self.result[0][2]
                self.bb.h2 = self.result[0][3]
                self.bb.l2 = self.result[0][4]

            else:
                self.bb.x3 = self.result[0][0]
                self.bb.y3 = self.result[0][1]
                self.bb.w3 = self.result[0][2]
                self.bb.h3 = self.result[0][3]
                self.bb.l3 = self.result[0][4]

            if self.result[1][4] == 0:

                self.bb.x1 = self.result[1][0]
                self.bb.y1 = self.result[1][1]
                self.bb.w1 = self.result[1][2]
                self.bb.h1 = self.result[1][3]
                self.bb.l1 = self.result[1][4]

            elif self.result[1][4] == 1:
                self.bb.x2 = self.result[1][0]
                self.bb.y2 = self.result[1][1]
                self.bb.w2 = self.result[1][2]
                self.bb.h2 = self.result[1][3]
                self.bb.l2 = self.result[1][4]

            elif self.result[1][4] == 2:
                self.bb.x3 = self.result[1][0]
                self.bb.y3 = self.result[1][1]
                self.bb.w3 = self.result[1][2]
                self.bb.h3 = self.result[1][3]
                self.bb.l3 = self.result[1][4]

            else:
                pass

            if self.result[2][4] == 0:

                self.bb.x1 = self.result[1][0]
                self.bb.y1 = self.result[1][1]
                self.bb.w1 = self.result[1][2]
                self.bb.h1 = self.result[1][3]
                self.bb.l1 = self.result[1][4]

            elif self.result[2][4] == 1:
                self.bb.x2 = self.result[2][0]
                self.bb.y2 = self.result[2][1]
                self.bb.w2 = self.result[2][2]
                self.bb.h2 = self.result[2][3]
                self.bb.l2 = self.result[2][4]

            elif self.result[2][4] == 2:
                self.bb.x3 = self.result[2][0]
                self.bb.y3 = self.result[2][1]
                self.bb.w3 = self.result[2][2]
                self.bb.h3 = self.result[2][3]
                self.bb.l3 = self.result[2][4]
            else:
                pass
            

        self.pub.publish(self.bb)
        print("Published")
        return image



    def give_output( self , frame, choice) :        
        

        self.predict( frame )
        #print("selected object : {}".format(self.result[choice][4]))
        #print(self.result)
        #print("self.result : ",self.result)

        frame = self.result[-1]

        #print("Frame : ", frame )

        xcenter, ycenter = frame.shape[1]/2, frame.shape[0]/2
        #print("xcenter ", xcenter )

        cv2.line( frame , (int(xcenter) , 0) , (int(xcenter) , int(2*ycenter)) , (255,0,0) , 5)
        cv2.line( frame , (0 , int(ycenter)) , (int(xcenter*2) , int(ycenter)) , (255,0,0) , 5)

        obj_coords = []

        for coords in self.result[:-1] :
            if coords[4]==choice :
                obj_coords = coords
        
        #print("obj_coords",obj_coords)
        try :

            self.x, self.y = obj_coords[0], obj_coords[1]
            self.w , self.h =  obj_coords[2], obj_coords[3]

            #print(self.x,self.y,self.w,self.h)
            obj_xcenter, obj_ycenter = self.x + self.w/2, self.y + self.h/2        
            linear_x , linear_y , linear_z = [0]*3 

            if xcenter > obj_xcenter + self.w/2 :
                linear_x = -1
            elif xcenter < obj_xcenter - self.w/2 :
                linear_x = 1
            else:
                linear_x = 0
            print('linear_x : ',linear_x)

            if ycenter > obj_ycenter + self.h/2 :
                linear_y = 1
            elif ycenter < obj_ycenter - self.h/2 :
                linear_y = -1
            else:
                linear_y = 0
            self.output = [ frame , linear_x , linear_y, self.w ]
            return self.output

        except Exception:
            #print(Exception)
            print("Object with ID {} not found".format(choice))
            return [ frame, None, None, None]



    def predict( self, frame ) :
        #LABELS = open(LABELS_FILE).read().strip().split("\n")
        LABELS = ["yellow", "cube", "arc"]
        np.random.seed(4)
        COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
        	dtype="uint8")


        net = cv2.dnn.readNetFromDarknet(CONFIG_FILE, WEIGHTS_FILE)
        '''if is_cuda:
            print("Attempty to use CUDA")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)'''

        #image = cv2.imread(INPUT_FILE)
        image = cv2.resize( frame, ( 640,640 ))
        (H, W) = image.shape[:2]

        

        # determine only the *output* layer names that we need from YOLO
        ln = net.getLayerNames()
        ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]


        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
        	swapRB=True, crop=False)
        net.setInput(blob)
        start = time.time()
        layerOutputs = net.forward(ln)
        end = time.time()


        #print("[INFO] YOLO took {:.6f} seconds".format(end - start))


        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []

        # loop over each of the layer outputs
        for output in layerOutputs:
        	# loop over each of the detections
        	for detection in output:
        		# extract the class ID and confidence (i.e., probability) of
        		# the current object detection
        		scores = detection[5:]
        		classID = np.argmax(scores)
        		confidence = scores[classID]

        		# filter out weak predictions by ensuring the detected
        		# probability is greater than the minimum probability
        		if confidence > CONFIDENCE_THRESHOLD:
        			# scale the bounding box coordinates back relative to the
        			# size of the image, keeping in mind that YOLO actually
        			# returns the center (x, y)-coordinates of the bounding
        			# box followed by the boxes' width and height
        			box = detection[0:4] * np.array([W, H, W, H])
        			(centerX, centerY, width, height) = box.astype("int")

        			# use the center (x, y)-coordinates to derive the top and
        			# and left corner of the bounding box
        			x = int(centerX - (width / 2))
        			y = int(centerY - (height / 2))

        			# update our list of bounding box coordinates, confidences,
        			# and class IDs
        			boxes.append([x, y, int(width), int(height)])
        			confidences.append(float(confidence))
        			classIDs.append(classID)

        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD,
        	CONFIDENCE_THRESHOLD)

        #print("len(idxs) : ", len(idxs))

        
        result = []
        # ensure at least one detection exists
        if len(idxs) > 0:
            coords = []
            
            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                coords = [x, y, w, h, classIDs[i]]

                result.append( coords )
                #print("appended")

                color = [int(c) for c in COLORS[classIDs[i]]]

                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)

                text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                1, color, 2)
                
        #if result[:-1]!=[] :
        result.sort(key = takefourth)
        print(result)
        #result.append( image )
        self.result = result
        #print("result : ",self.result)
        #print("Result : ", self.result)
        return self.result, image

def dec_brightness(img, value = 0) :
    hsv = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )
    h, s, v = cv2.split(hsv)

    lim = value
    v[v<lim] = 0
    v[v>= lim] -= value

    final_hsv = cv2.merge((h,s,v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img


def takefourth(l) :
    return l[4]

rospy.init_node("yolo_publisher", anonymous = True)

while True :
    
    try :
        ret, frame = cap.read()
        if ret :
            dnn = DNN()
            image = dnn.publish(dec_brightness(frame,30))
            cv2.line(image , (320 , 0) , (320 , 640) , (255,0,0) , 3)
            cv2.line(image , (0 , 320) , (640 , 320) , (255,0,0) , 3)
            cv2.putText(image, " Yellow : 0", (25,25), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA, False)

            cv2.putText(image, " Cube : 1", (25,50), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA, False)

            cv2.putText(image, " Arc : 2", (25,75), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA, False)

            #print(image.shape)
            cv2.imshow("image", image)
            if cv2.waitKey(1) & 0xFF == ord('q') :
                break
    except KeyboardInterrupt:
        print("error")
        cap.release()
        cv2.destroyAllWindows()