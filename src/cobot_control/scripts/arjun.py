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

cap = cv2.VideoCapture(-1)

class ImageProcessing:
  def __init__(self, lower ,upper ):
    self.lower = lower  # The lower limit of the colour range in the form of RGB Values
    self.upper = upper  # The Upper limit of the colour range in the form of RGB Values
    self.radius = None  # The radius of the Circle drawn around the object
    self.center = None  # The center of the circle drawn around the object
    self.x,self.y,self.w,self.h = [0]*4 
    
  def publish(self , frame):
    self.frame=frame
    blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, self.lower, self.upper)
    #mask = cv2.erode(mask, None, iterations=2)
    #mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    result = [self.frame , None , None , mask, self.w]

    if len(cnts) > 0:
      c = max(cnts, key=cv2.contourArea)
      self.x, self.y , self.w , self.h = cv2.boundingRect(c)
      #print("WIDTH : ",self.w)
      cv2.rectangle(frame , (self.x , self.y) , (self.x + self.w , self.y + self.h) , (36,255,12) , 2)
      
      xcenter = self.frame.shape[1]/2
      ycenter = self.frame.shape[0]/2
      obj_xcenter = self.x + self.w/2
      obj_ycenter = self.y + self.h/2
      
      cv2.line(self.frame , (int(xcenter) , 0) , (int(xcenter) , int(2*ycenter)) , (255,0,0) , 5)
      cv2.line(self.frame , (0 , int(ycenter)) , (int(xcenter*2) , int(ycenter)) , (255,0,0) , 5)
      cv2.circle(self.frame , (int(obj_xcenter) , int(obj_ycenter)) , 5 , (0,0,255) , -1)
      
      linear_x , linear_y , linear_z = [0]*3 
      
      if xcenter > obj_xcenter + self.w/2 :
        linear_x = -1
      elif xcenter < obj_xcenter - self.w/2 :
        linear_x = 1
      else:
        linear_x = 0
        
      if ycenter > obj_ycenter + self.h/2 :
        linear_y = 1
      elif ycenter < obj_ycenter - self.h/2 :
        linear_y = -1
      else:
        linear_y = 0
      result = [self.frame , linear_x , linear_y , mask, self.w]
      
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
      pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
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
        #print("count : ",self.count_x)
        
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
    self.pick_down = [j1,j2,j3,j4,j5,j6]
    print("fix_error_y : ",self.pick_down)
    return False , [j1,j2,j3,j4,j5,j6]

  def fix_error_orient(self, result, pose) :
    [j1,j2,j3,j4,j5,j6] = pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]
    w = result[-1]
    print("w : ",w)

    if w > WIDTH_THRESH :
      j6 += 0.7
      self.rotate_iters += 1
      self.go_to_joint_state(j1,j2,j3,j4,j5,j6)
      print("Fixing orientation error")
      time.sleep(0.5)      
    
    else:
      print("Orientation error fixed")
      self.pick_down = [j1,j2,j3,j4,j5,j6]
      return( False , [j1,j2,j3,j4,j5,j6] )      

    return ( True , [j1,j2,j3,j4,j5,j6] )

  
  def fix_error(self,start_pose) :
        
    flag_x= True  
    flag_orient = True
    flag_y = True
    ip = ImageProcessing((20, 100 , 100 ),( 30 , 255, 255))
    
    loop = 0
    while (cap.isOpened() ) and ( flag_x or flag_y or flag_orient):
      loop += 1
      ret , frame = cap.read()
      if ret :
        
        result=ip.publish(frame)
        if flag_orient ==  True :
          flag_orient, start_pose = self.fix_error_orient(result, start_pose)         
          

        if flag_orient == False and flag_x == True :
          flag_x, start_pose = self.fix_error_x(result , start_pose)

        if flag_orient == False and flag_x == False and flag_y ==  True :
          flag_y, start_pose = self.fix_error_y(result , start_pose)
          

        cv2.imshow("mask" , result[-1])
        cv2.imshow("frame" , frame)
        
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

    ur5.go_to_joint_state(ur5.pick_down[0] - ur5.rotate_iters*0.1 , ur5.pick_down[1] + 23 , ur5.pick_down[2] - 23, ur5.pick_down[3] , ur5.pick_down[4]  ,ur5.pick_down[5])
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
