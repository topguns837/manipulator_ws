#!/usr/bin/env python

import sys
import time
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
import tf2_msgs.msg
import tf
import numpy as np
#from mask import ImageProcessing
import cv2
import imutils
#goalList=[[-185 , 36 , 54, 172 , -91 , -181],[-145 , 42 , 54 , 172 , -90 , -181],[-145,48, 54 ,167 ,-90 ,-181]]

pick_down = [-185 , 36 , 54, 172 , -91 , -181]
WIDTH_THRESH = 120

INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.2
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.4

colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

is_cuda = len(sys.argv) > 1 and sys.argv[1] == "cuda"

#net = build_model(is_cuda)
#capture = load_capture()

start = time.time_ns()
frame_count = 0
total_frames = 0
fps = -1


cap = cv2.VideoCapture(-1)

class WorkpieceDetector :
    def __init__(self):
        self.frame_count = 0
        self.total_frames = 0
        self.fps = -1
        self.start = time.time_ns()
        self.class_list = [ "Yellow", "Cube", "Arc" ]

    def build_model(self , is_cuda):
        self.net = cv2.dnn.readNet("best_epoch_50.onnx")
        if is_cuda:
            print("Attempty to use CUDA")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return self.net



    def detect(self,image,net ):
        self.net = net
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
        self.net.setInput(blob)
        preds = self.net.forward()
        return preds

    def load_capture(self):
        self.capture = cv2.VideoCapture(3)
        return self.capture

    def load_classes(self):
        self.class_list = [ "Yellow", "Cube", "Arc" ]
        #with open("classes.txt", "r") as f:
            #self.class_list = [cname.strip() for cname in f.readlines()]
        return self.class_list



    def wrap_detection(self,input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / INPUT_WIDTH
        y_factor =  image_height / INPUT_HEIGHT

        for r in range(rows):
            row = output_data[r]
            confidence = row[4]
            if confidence >= 0.4:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes

    def format_yolov5(self,frame):

        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        return result


    def control_loop(self, frame) :
        #frame = self.format_yolov5(frame)
        self.net = self.build_model(is_cuda)
        self.capture = self.load_capture()  
        
        
        if frame is None:
            print("Frame is None")
            return [ frame, 0, 0 ]

        else :
            inputImage = self.format_yolov5(frame)
            resized = cv2.resize(inputImage , (640,640))
            blurred = cv2.blur(resized ,(10,10))
            outs = self.detect(blurred, self.net)

            class_ids, confidences, boxes = self.wrap_detection(blurred, outs[0])
            print("ID : " , class_ids)
            print("Boxes : ",boxes)

            self.frame_count += 1
            self.total_frames += 1
            for (classid, confidence, box) in zip(class_ids, confidences, boxes):
                color = colors[int(classid) % len(colors)]
                cv2.rectangle(resized, box, color, 2)
                cv2.rectangle(resized, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
                #try :
                cv2.putText(resized, self.class_list[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
                #except :
                    #pass
                 
            if self.frame_count >= 30:
                self.end = time.time_ns()
                self.fps = 1000000000 * frame_count / (self.end - self.start)
                self.frame_count = 0
                self.start = time.time_ns()

            '''if self.fps > 0:
                self.fps_label = "FPS: %.2f" % self.fps
                cv2.putText(frame, fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)'''

            result = [ resized, 0, 0 ]

            self.x, self.y , self.w , self.h = boxes[0][0], boxes[0][1], boxes[0][2], boxes[0][3]

            cv2.line(self.frame , (int(xcenter) , 0) , (int(xcenter) , int(2*ycenter)) , (255,0,0) , 5)
            cv2.line(self.frame , (0 , int(ycenter)) , (int(xcenter*2) , int(ycenter)) , (255,0,0) , 5)
            cv2.circle(self.frame , (int(obj_xcenter) , int(obj_ycenter)) , 5 , (0,0,255) , -1)

            xcenter = resized.shape[1]/2
            ycenter = resized.shape[0]/2

            obj_xcenter = self.x + self.w/2
            obj_ycenter = self.y + self.h/2

            linear_x , linear_y , linear_z = [0]*3 

            if xcenter > obj_xcenter + self.w/2 :
                linear_x = -1
                #self.velocity_msg.linear.x = -1
            elif xcenter < obj_xcenter - self.w/2 :
                linear_x = 1
                #self.velocity_msg.linear.x = 1
            else:
                #self.velocity_msg.linear.x = 0
                linear_x = 0

            if ycenter > obj_ycenter + self.h/2 :
                linear_y = 1
                #self.velocity_msg.linear.y = 1
            elif ycenter < obj_ycenter - self.h/2 :
                linear_y = -1
                #self.velocity_msg.linear.y = -1
            else:
                linear_y = 0
                #self.velocity_msg.linear.y = 0
                # #self.pub.publish(self.velocity_msg)
                # 
            result = [ resized , linear_x , linear_y, self.w ]
            return result

        
        
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

        self.rotate_iters = 0

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

    def fix_error_x(self , result ,  pose):
         

        print("result[1] : ",result[1])
        [j1,j2,j3,j4,j5,j6] = pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]

        frame = result[0]

        dir = 0
        if result[1] ==  None :
            pass
        else:


            if result[1] == 1:

              
                dir = +1
                j1 -= 0.2
                self.count_x += 1
                self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
                time.sleep(0.5)

            elif result[1] == -1 :

                dir = -1
                j1 += 0.2
                self.count_x += 1
                self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
                time.sleep(0.5)

            elif result[1] == 0 :
                

                j5 += self.count_x*(0.2)*dir
                self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
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

    def fix_error_orient(self, result, pose) :
        [j1,j2,j3,j4,j5,j6] = pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]
        w = result[-1]

        if self.rotate_iters == 0:
            self.init_width = w
        #print("w : ",w)

        if w > WIDTH_THRESH :

          #print("rotate_iter : ",self.rotate_iters)
            j6 += 0.7*self.rotate_dir
            self.rotate_iters += 1
            self.go_to_joint_state(j1,j2,j3,j4,j5,j6)

            if self.rotate_dir == 1 and self.rotate_iters > 5 and w > self.init_width :

              self.rotate_dir = -1
              self.rotate_iters = 0
              self.go_to_joint_state(-185 , 36 , 54, 172 , -91 , -181)

              print("DIR CHANGE")
              time.sleep(0.5)


            #print("Fixing orientation error")
            time.sleep(0.5)      

            else:
                #print("Orientation error fixed")
                self.pick_down = [j1,j2,j3,j4,j5,j6]
                return( False , [j1,j2,j3,j4,j5,j6] )
            return ( True , [j1,j2,j3,j4,j5,j6] )
        
    def fix_error(self,start_pose) :
        
        flag_x= True  
        flag_orient = True
        flag_y = False
        wd = WorkpieceDetector()

        loop = 0
        while (cap.isOpened() ) and ( flag_x or flag_y or flag_orient):

            loop += 1
            ret , frame = cap.read()

            if ret :
                result=wd.control_loop(frame)

                if result[-1]==result[-2]==None :
                    pass
                else:

                    if flag_orient ==  True :
                        flag_orient, start_pose = self.fix_error_orient(result, start_pose)        


                    if flag_orient == False and flag_x == True :
                        flag_x, start_pose = self.fix_error_x(result , start_pose)

                    if flag_orient == False and flag_x == False and flag_y ==  True :
                        flag_y, start_pose = self.fix_error_y(result , start_pose)


              #cv2.imshow("mask" , result[-1])
            cv2.imshow("frame" , result[0])
            cv2.waitKey(1)

        #pick_down = start_pose
        #print("pick_down : ",pick_down)


        cap.release()
        cv2.destroyAllWindows()
        cv2.waitKey(1)
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

        ur5.go_to_joint_state(ur5.pick_down[0] - ur5.rotate_iters*0.1 , ur5.pick_down[1], ur5.pick_down[2], ur5.pick_down[3] , ur5.pick_down[4]  ,ur5.pick_down[5])
        print("Pick Down : ",ur5.pick_down)
        time.sleep(5)

        ur5.go_to_joint_state(ur5.pick_down[0] - ur5.rotate_iters*0.1 , ur5.pick_down[1] + 21 , ur5.pick_down[2] - 21, ur5.pick_down[3] , ur5.pick_down[4]  ,ur5.pick_down[5])
        print("Pick Down")
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