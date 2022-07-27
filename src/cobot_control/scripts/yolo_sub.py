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
from cobot_control.msg import boundingboxes

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




def dec_brightness(img, value = 0) :
    hsv = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )
    h, s, v = cv2.split(hsv)

    lim = value
    v[v<lim] = 0
    v[v>= lim] -= value

    final_hsv = cv2.merge((h,s,v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img
'''
class DNN :
    def __init__(self) :
        self.H, self.W = 0, 0
        self.result = []
        self.output = []
        

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
        if is_cuda:
            print("Attempty to use CUDA")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

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
        result.append( image )
        self.result = result
        #print("result : ",self.result)
        #print("Result : ", self.result)
        return self.result
        '''


def takefourth(l) :
    return l[4]









        
class UR5:
    def trajectory_execution_status(self,goal_pose):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        flag=0
        rospy.loginfo("===========Robot in motion. Please wait=============")
        while(flag == 0):
            try :
                (trans,rot) = listener.lookupTransform('base', 'tool0', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            x_cord = round(trans[0],3)
            y_cord = round(trans[1],3)
            z_cord = round(trans[2],3)
            rx = -round(rot[0],3)
            ry = -round(rot[1],3)
            rz = -round(rot[2],3)
            diff_x = x_cord - (goal_pose.position.x)
            diff_y = y_cord - (goal_pose.position.y)
            diff_z = z_cord - (goal_pose.position.z)
            diff_rx = rx - (goal_pose.orientation.x)
            diff_ry = ry - (goal_pose.orientation.y)
            diff_rz = rz - (goal_pose.orientation.z)
            rospy.loginfo(diff_x)
            rospy.loginfo(diff_y)
            rospy.loginfo(diff_z)
            # rospy.loginfo(diff_rx)
            # rospy.loginfo(diff_ry)
            # rospy.loginfo(diff_rz)
            if(-0.01<diff_x<0.5 and -0.01<diff_y<0.5 and -0.01<diff_z<0.5):

            # if(-1.5<diff_x<1.5 and -1.5<diff_y<1.5 and -1.5<diff_z<1.5):
                flag = 1
                rospy.loginfo("Successfully reached")
            #else:
                #rospy.loginfo("Robot in Motion.... please wait.......")


    def __init__(self):
        
        #self.bridge = CvBridge()
        #self.pub = rospy.Publisher("yolo_image", Image, queue_size = 10)
        self.sub = rospy.Subscriber("/bounding_box", boundingboxes, self.bb_callback)
        self.rotate_iters = 0
        self.rotate_dir = 1
        self.init_width = 0
        self.orient_start_iters = 0

        self.dir_x,self.dir_y = 0,0
        self.bb = None

        #self.pick_down = [-185 , 36 , 54, 172 , -91 , -181]

        self.pick_down = [-185 , 36 , 54, 172 , -91 , -181]
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        print (robot.get_current_state())
        self.count_x , self.count_y = 0 , 0

        self.move_group = move_group

    def bb_callback(self, bb) :
        #print("callback")
        self.bb = bb

    def give_coords(self, choice) :       


        if choice == 0 :
            if self.bb.x1==self.bb.y1==self.bb.w1==self.bb.h1==0.0 :
                pass
            else:
                obj_coords = [ self.bb.x1, self.bb.y1, self.bb.w1, self.bb.h1 ]

        elif choice == 1 :
            if self.bb.x2==self.bb.y2==self.bb.w2==self.bb.h2==0.0 :
                pass
            else:
                obj_coords = [ self.bb.x2, self.bb.y2, self.bb.w2, self.bb.h2 ]

        elif choice == 2 :
            if self.bb.x3==self.bb.y3==self.bb.w3==self.bb.h3==0.0 :
                pass
            else:
                obj_coords = [ self.bb.x3, self.bb.y3, self.bb.w3, self.bb.h3 ]

        else:
            pass
        self.obj_coords = obj_coords
        #print("obj_coords : ", obj_coords)





        #try :
        xcenter, ycenter = 320,320
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
        self.output = [ linear_x , linear_y, self.w ]
        return self.output

        '''except Exception:
            #print(Exception)
            print("Object with ID {} not found".format(choice))
            return [ None, None, None]'''




    def go_to_pose_goal(self,arg_pose):
        move_group = self.move_group
        rospy.loginfo(arg_pose)
        move_group.set_pose_target(arg_pose)
        plan = move_group.go(wait=True)
        if (plan == True):

            rospy.loginfo(
               '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
               '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')
        move_group.stop()
        move_group.clear_pose_targets()

    def go_to_joint_state(self,x,y,z,rx,ry,rz):

        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0]=round(math.radians(x),2)
        joint_goal[1]=round(math.radians(y),2)
        joint_goal[2]=round(math.radians(z),2)
        joint_goal[3]=round(math.radians(rx),2)
        joint_goal[4]=round(math.radians(ry),2)
        joint_goal[5]=round(math.radians(rz),2)
    
        # joint_goal[0]=x
        # joint_goal[1]=y
        # joint_goal[2]=z
        # joint_goal[3]=rx
        # joint_goal[4]=ry
        # joint_goal[5]=rz


        # if(joint_goal[0]<0):
        #   joint_goal[0] = joint_goal[0] + pi
        # else:
        #   joint_goal[0] = joint_goal[0] - pi

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group

        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL



    def gripper_motion(self,grip):
        ub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
        r = rospy.Rate(10) #10hz
        msg = GripperCtrl()
        msg.initialize = True
        n=0
        if grip == 0:
          msg.position = 0
          msg.force = 100
          msg.speed = 100
        if grip == 1:
          msg.position = 1000
          msg.force = 100
          msg.speed = 100
        if grip == 2:
          msg.position = input("Enter Position")
          msg.force = 100
          msg.speed = 100
        pub.publish(msg)

    def fix_pos_x(self, result, pose, choice) :

        [j1,j2,j3,j4,j5,j6] = pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]
        #dnn_pos_x = DNN()
        #coords = self.give_coords(choice)
        #coords = result[choice]
        print("pos_x_coords : ", self.obj_coords)
        pos_x_error = (self.obj_coords[0] + self.obj_coords[2]/2) - 320

        j1 -= (0.028)*pos_x_error
        j6 -= (0.028)*pos_x_error
        #j5 -= pos_x_error

        self.go_to_joint_state(j1,j2,j3,j4,j5,j6)

        return ( False , [j1,j2,j3,j4,j5,j6] )


    def fix_error_x(self , result ,  pose):        

        
        [j1,j2,j3,j4,j5,j6] = pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]

        #frame = result[0]

        self.orient_dir = 0
        '''if result[1] ==  None :
            pass
        else:'''
        if result[1] == 1:
          
            self.orient_dir = +1
            j1 -= 0.2
            self.count_x += 1
            self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
            time.sleep(0.5)
        elif result[1] == -1 :
            self.orient_dir = -1
            j1 += 0.2
            self.count_x += 1
            self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
            time.sleep(0.5)
        elif result[1] == 0 :                
            #j5 += self.count_x*(0.2)*self.orient_dir
            self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
            time.sleep(0.5)
            self.pick_down = [j1,j2,j3,j4,j5,j6]
            print("X error fixed")
            return ( False , [j1,j2,j3,j4,j5,j6] )
        else:
            pass     
        
        
        return( True , [j1,j2,j3,j4,j5,j6] )


    
    def fix_error_y(self , result ,  pose):

        [j1,j2,j3,j4,j5,j6] = pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]  


        if result[2]==None:
            pass
        else:
            if result[2] == 1:
                self.dir_y= +1
                j2 += 0.2
                self.count_y += 1
                self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
                time.sleep(0.5)

            elif result[2] == -1 :
                self.dir_y = -1
                j2 -= 0.2
                self.count_y += 1
                self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
                time.sleep(0.5)

            elif result[2] == 0 :
                BASE_ADJ =  self.count_y*(0.2)*self.dir_y
                print(BASE_ADJ)
                j3 += self.count_y*(0.2)*(-self.dir_y)
                #BASE_ADJ = self.count_y*(0.2)*dir*(-1)
                #self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
                self.pick_down = [j1,j2,j3,j4,j5,j6]
                time.sleep(0.5)
                print("Y error fixed")
                return ( False , [j1,j2,j3,j4,j5,j6] )

            else:
                pass

        self.pick_down = [j1,j2,j3,j4,j5,j6]
        #print("fix_error_y : ",self.pick_down)
        return True , [j1,j2,j3,j4,j5,j6]

    def fix_orient_start( self, result, pose, choice) :
        

        [j1,j2,j3,j4,j5,j6] = pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]
        w = result[-1]
        self.orient_start_fix = (0.25)*(w-WIDTH_THRESH[choice])

        if self.orient_start_iters == 0:
            self.init_width = w
            print("init_width : ", self.init_width)
            if w > WIDTH_THRESH[choice] :  
                print("start_orient")    
                j6 += self.orient_start_fix
                self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
                self.orient_start_iters += 1
                time.sleep(10)
                return ( True , [j1,j2,j3,j4,j5,j6] )
                

            else :
                return( False , [j1,j2,j3,j4,j5,j6] )

        elif self.orient_start_iters == 1 :
            if w + 5 > self.init_width :
                print("init_width : ", self.init_width)
                print("current width : ", w)
                self.orient_start_iters += 1
                return ( True , [j1,j2,j3,j4,j5,j6] )
            else:
                self.rotate_iters = int(self.orient_start_fix/1.5)
                self.rotate_dir = 1
                return( False , [j1,j2,j3,j4,j5,j6] )
                
        
        elif self.orient_start_iters == 2 :
            print("reversing")
            
            j6 -= 2*self.orient_start_fix
            self.go_to_joint_state(j1,j2,j3,j4,j5,j6 )
            self.rotate_iters = int(self.orient_start_fix/1.5)
            self.rotate_dir = -1
            return( False , [j1,j2,j3,j4,j5,j6] )


        else:
            return ( True , [j1,j2,j3,j4,j5,j6] )  

        return ( True , [j1,j2,j3,j4,j5,j6] )     
                
                
        


    def fix_error_orient(self, result, pose, choice) :
        [j1,j2,j3,j4,j5,j6] = pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]
        w = result[-1]
        
        
        if self.rotate_iters == 0:
            self.init_width = w
            #print("init_width : ", self.init_width)
        print("w : ",w)

        if w > WIDTH_THRESH[choice] :

            #print("rotate_iter : ",self.rotate_iters)
            j6 += 1.5*self.rotate_dir
            self.rotate_iters += 1
            self.go_to_joint_state(j1,j2,j3,j4,j5,j6)

            '''if self.rotate_dir == 1 and self.rotate_iters >= 15 and w > self.init_width :

              self.rotate_dir = -1
              self.rotate_iters = 0
              self.go_to_joint_state(-185 , 36 , 54, 172 , -91 , -181)
              j1,j2,j3,j4,j5,j6 = -185 , 36 , 54, 172 , -91 , -181

              print("DIR CHANGE")
              time.sleep(0.5)'''

        else:
            print("Orientation error fixed")
            self.pick_down = [j1,j2,j3,j4,j5,j6]
            return( False , [j1,j2,j3,j4,j5,j6] )

        return ( True , [j1,j2,j3,j4,j5,j6] )
        
    def fix_error(self,start_pose) :
        
        flag_x= True  
        flag_orient = True
        flag_orient_start = True
        flag_y = False
        flag_pos_x = True
        #dnn = DNN()
        choice = 0

        loop = 0
        choice_flag = True
        while ( flag_x or flag_y or flag_orient):

            loop += 1
            #ret , frame = cap.read()
            #frame = dec_brightness(frame, 30)

            if choice_flag==True :
                #result_choice = dnn.predict( frame )
                #print("result_choice : ",result_choice)
                #choices = len( result_choice[:-1] )

                #print("Available objects are : ")
                #for c in range(choices) :
                    #print("{} : {}".format(result_choice[c][4], LABELS[result_choice[c][4]]))
                choice = int(input("Enter your choice : "))
                choice_flag = False

            if True :
                result=self.give_coords( choice)
                print("result : ", result)
                
                if result[-1]==result[-2]==result[-3]==0.0 :
                    pass
                else:
                    if flag_pos_x == True :
                        flag_pos_x, start_pose = self.fix_pos_x( result, start_pose, choice )
                        
                    if flag_orient_start == True and flag_pos_x == False :
                        #print(result, start_pose, choice)
                        flag_orient_start, start_pose = self.fix_orient_start( result, start_pose, choice )

                    if flag_orient ==  True and flag_pos_x ==  False and flag_orient_start == False:
                        flag_orient, start_pose = self.fix_error_orient(result, start_pose, choice)        


                    if flag_orient == False and flag_orient_start == False and flag_pos_x == False and flag_x == True :
                        flag_x, start_pose = self.fix_error_x(result , start_pose)

                    if flag_orient == False and flag_orient_start == False and flag_x == False and flag_pos_x == False and flag_y ==  True :
                        flag_y, start_pose = self.fix_error_y(result , start_pose)


              
            #cv2.imshow("frame" , result[0])
            #cv2.waitKey(1)

        #cap.release()
        #cv2.destroyAllWindows()
        #cv2.waitKey(1)
        return
      
def main():
    try:
        goalList=[[-185 , 36 , 54, 172 , -91 , -181],[-145 , 42 , 54 , 172 , -90 , -181],[-145,48, 54 ,167 ,-90 ,-181]]
        [j01,j02,j03,j04,j05,j06] = goalList[0][0],goalList[0][1],goalList[0][2],goalList[0][3],goalList[0][4],goalList[0][5]
        [j11,j12,j13,j14,j15,j16] = goalList[1][0],goalList[1][1],goalList[1][2],goalList[1][3],goalList[1][4],goalList[1][5]

        [j31,j32,j33,j34,j35,j36] = goalList[2][0],goalList[2][1],goalList[2][2],goalList[2][3],goalList[2][4],goalList[2][5]

        ur5=UR5()


        ur5.go_to_joint_state(j01,j02,j03,j04,j05,j06)
        print("Pick Up")
        time.sleep(5)

        ur5.fix_error(goalList[0])
        print("Alligned")
        time.sleep(5)

        ur5.go_to_joint_state(ur5.pick_down[0] -  ur5.rotate_dir*ur5.rotate_iters*(0.15) , ur5.pick_down[1], ur5.pick_down[2], ur5.pick_down[3] , ur5.pick_down[4]  ,ur5.pick_down[5])
        print("Pick Down : ",ur5.pick_down)
        time.sleep(5)

        ur5.go_to_joint_state(ur5.pick_down[0] - ur5.rotate_dir*ur5.rotate_iters*(0.15) , ur5.pick_down[1] + 21  , ur5.pick_down[2] - 21, ur5.pick_down[3] , ur5.pick_down[4]  ,ur5.pick_down[5])
        print("Pick Down")
        time.sleep(5)

        ur5.go_to_joint_state(j01,j02,j03,j04,j05,j06)
        print("Pick Up")
        time.sleep(5)

        ur5.go_to_joint_state(j11,j12,j13,j14,j15,j16)
        print("Drop Up")
        time.sleep(5)

        #ur5.go_to_joint_state(j31,j32,j33,j34,j35,j36)


    

    except rospy.ROSInterruptException:

        return
    except KeyboardInterrupt:
        cap.release()
        cv2.destroyallwindows()   
    
        return

if __name__ == '__main__':
    main()