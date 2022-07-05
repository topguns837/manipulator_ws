#!/usr/bin/env python
import sys
import time
from turtle import goto
import roslib
from math import pi
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from dh_gripper_msgs.msg import GripperCtrl
from moveit_commander.conversions import pose_to_list
import tf2_msgs.msg
import tf
import cv2
import numpy as np

class UR5:

  def trajectory_execution_status(self,goal_pose):
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    flag=0
    rospy.loginfo("===========Robot in motion. Please wait=============")
    while(flag == 0):
        try:
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
  
    move_group = self.move_group

    
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
      
def main():

  try:
    goalList=[[-176,-23,141,61,-85,-180],[-175,20,124,36,-84,-180],[-179,-5,92,89,-85,-177],[-89,-13,92,103,-85,-177]]
    

    # ur5=UR5()
    for id,coord in enumerate(goalList):
      ur5=UR5()
      # x=round(math.radians(coord[0]),2)
      # y=round(math.radians(coord[1]),2)
      # z=round(math.radians(coord[2]),2)
      # rx=round(math.radians(coord[3]),2)
      # ry=round(math.radians(coord[4]),2)
      # rz=round(math.radians(coord[5]),2)
      x = coord[0]
      y = coord[1]
      z = coord[2]
      rx = coord[3]
      ry = coord[4]
      rz = coord[5]
    
      ur5.go_to_joint_state(x,y,z,rx,ry,rz)
      time.sleep(2)

    cap=cv2.VideoCapture(0)
    
    X=[210,170,137,119,103,92,86,76,71,66,60]
    Y=[20,25,30,35,40,45,50,55,60,65,70]

    coff=np.polyfit(X,Y,2)
    error =2 

    j1 = goalList[3][0]
    j2 = goalList[3][1]
    j3 = goalList[3][2]
    j4 = goalList[3][3]
    j5 = goalList[3][4]
    j6 = goalList[3][5]

    temp=0
    count=0
    
    while temp==0:
      ret,img=cap.read()
      imgHSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
      H,W,c=img.shape
      pinkLower=np.array([160,100,100])
      pinkUpper=np.array([255,255,255])
      hsvMask=cv2.inRange(imgHSV,pinkLower,pinkUpper)
      _,contours,_=cv2.findContours(hsvMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
      if contours==[]:
        j1+=1
        ur5.go_to_joint_state(j1, j2, j3, j4, j5, j6)

      for cont in contours:
        
        area=cv2.contourArea(cont)
        cx,pos=0,0
        
        if area>600:
          
          x,y,w,h=cv2.boundingRect(cont)
          x1,y1=int(x),int(h/2)
          x2,y2=int(x+w),int(h/2)

          cv2.circle(img,(x1,y1+y),2,(200,0,59),2)
          cv2.circle(img,(x2,y2+y),2,(200,0,59),2)
          cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
          cx,cy=int(x+(w/2)),int(y+(h/2))
          c1=str(cx)+','+str(cy)
          cv2.circle(img,(cx,cy),2,(0,255,0),2)
          cv2.putText(img,c1,(cx+1,cy+1),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,255,0),1)
          
          cfx,cfy=int(W/2),int(H/2)
          c2=str(cfx)+','+str(cfy)
          cv2.circle(img,(cfx,cfy),2,(0,255,0),2)
          cv2.putText(img,c2,(cfx+1,cfy+1),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,255,0),1)
          distance=int(math.sqrt(((y2-y1)**2)+((x2-x1)**2)))
          A,B,C=coff
          distCM=int(A*distance**2+B*distance+C)+error

          if cx< cfx and temp==0:
            j1 = j1+1
            count+=1
            # j5 = j5+1
            ur5.go_to_joint_state(j1, j2, j3, j4, j5, j6)
            # time.sleep(0.5)
          
            if cx < cfx+50 and cx > cfx-50:
              temp=1

          if temp==1:
            j5=j5+count
            ur5.go_to_joint_state(j1, j2, j3, j4, j5, j6)
     
            # ur5.go_to_joint_state(-175,20,124,36,-84,-180)
            
      cv2.imshow('image',img)
      cv2.waitKey(1)
    cap.release()
    # cv2.destroyallwindows()   
    time.sleep(7)


    # goalList2=[[j1,-29,108,99,j5,-181],[j1,-6,95,90,j5,-181],[j1,17,60,102,j5,-181],[j1,-4,83,102,j5,-181],[j1,7,89,82,j5,-181],[j1,17,74,88,j5,-181]]
    # goalList2=[[j1,2,76,103,j5,-181],[j1,6,89,83,j1,-181],[j1,19,75,84,j5,-181],[j1,-7,101,84,j5,-181],[j1,-6,95,90,j5,-181]]
    goalList2=[[j1,2,76,103,j5,-181],[-73,-8,106,79,-73,-181],[-77,14,76,92,-78,-181],[j1,2,76,103,j5,-181]]
    ur5=UR5()
    for id,pt in enumerate(goalList2):
      x0 = pt[0]
      y0 = pt[1]
      z0 = pt[2]
      rx0 = pt[3]
      ry0 = pt[4]
      rz0 = pt[5]
      ur5.go_to_joint_state(x0,y0,z0,rx0,ry0,rz0)
      time.sleep(5)
    

  except rospy.ROSInterruptException:

    return
  except KeyboardInterrupt:
    cap.release()
    cv2.destroyallwindows()   
    
    return

if __name__ == '__main__':
  main()
