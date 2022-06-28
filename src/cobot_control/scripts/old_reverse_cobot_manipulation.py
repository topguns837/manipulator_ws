#!/usr/bin/env python
import sys
import time
import roslib
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from dh_gripper_msgs.msg import GripperCtrl
from moveit_commander.conversions import pose_to_list
import tf2_msgs.msg
import tf


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
        #rospy.loginfo(diff_x)
        #rospy.loginfo(diff_y)
        #rospy.loginfo(diff_z)
        #rospy.loginfo(diff_rx)
        #rospy.loginfo(diff_ry)
        #rospy.loginfo(diff_rz)
        #if(-0.005<diff_x<0.005 and -0.005<diff_y<0.005 and 0.113<diff_z<0.119 and -0.005<diff_rx<0.005 and -0.005<diff_ry<0.005 and -0.005<diff_rz<0.005):
        if(-0.005<diff_x<0.005 and -0.005<diff_y<0.005 and 0.113<diff_z<0.119):
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
    print robot.get_current_state()

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


  def gripper_motion(self,grip):
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    r = rospy.Rate(10) #10hz
    msg = GripperCtrl()
    msg.initialize = False
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
    while n<10:
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()
        n=n+1


def main():
  try:
      ur5_pose_0 = geometry_msgs.msg.Pose()
      ur5_pose_0.position.x = 0.6
      ur5_pose_0.position.y = 0.145
      ur5_pose_0.position.z = 0.2
      quaternion = tf.transformations.quaternion_from_euler(-3.14,0,1.54)
      ur5_pose_0.orientation.x = quaternion[0]
      ur5_pose_0.orientation.y = quaternion[1]
      ur5_pose_0.orientation.z = quaternion[2]
      ur5_pose_0.orientation.w = quaternion[3]

      ur5_pose_1 = geometry_msgs.msg.Pose()
      ur5_pose_1.position.x = 0.6
      ur5_pose_1.position.y = 0.145
      ur5_pose_1.position.z = 0.15
      quaternion = tf.transformations.quaternion_from_euler(-3.14,0,1.54)
      ur5_pose_1.orientation.x = quaternion[0]
      ur5_pose_1.orientation.y = quaternion[1]
      ur5_pose_1.orientation.z = quaternion[2]
      ur5_pose_1.orientation.w = quaternion[3]

      ur5_pose_2 = geometry_msgs.msg.Pose()
      ur5_pose_2.position.x = 0.190
      ur5_pose_2.position.y = 0.145
      ur5_pose_2.position.z = 0.3
      quaternion = tf.transformations.quaternion_from_euler(-3.14,0,1.54)
      ur5_pose_2.orientation.x = quaternion[0]
      ur5_pose_2.orientation.y = quaternion[1]
      ur5_pose_2.orientation.z = quaternion[2]
      ur5_pose_2.orientation.w = quaternion[3]

      ur5_pose_5 = geometry_msgs.msg.Pose()
      ur5_pose_5.position.x = 0.140
      ur5_pose_5.position.y = 0.2
      ur5_pose_5.position.z = 0.3
      quaternion = tf.transformations.quaternion_from_euler(3.14,0,3.14)
      ur5_pose_5.orientation.x = quaternion[0]
      ur5_pose_5.orientation.y = quaternion[1]
      ur5_pose_5.orientation.z = quaternion[2]
      ur5_pose_5.orientation.w = quaternion[3]

      ur5_pose_3 = geometry_msgs.msg.Pose()
      ur5_pose_3.position.x = -0.190
      ur5_pose_3.position.y = 0.145
      ur5_pose_3.position.z = 0.2
      quaternion = tf.transformations.quaternion_from_euler(3.14,0,-1.54)
      ur5_pose_3.orientation.x = quaternion[0]
      ur5_pose_3.orientation.y = quaternion[1]
      ur5_pose_3.orientation.z = quaternion[2]
      ur5_pose_3.orientation.w = quaternion[3]

      ur5_pose_4 = geometry_msgs.msg.Pose()
      ur5_pose_4.position.x = -0.365
      ur5_pose_4.position.y = 0
      ur5_pose_4.position.z = 0.2
      quaternion = tf.transformations.quaternion_from_euler(3.14,0,-1.54)
      ur5_pose_4.orientation.x = quaternion[0]
      ur5_pose_4.orientation.y = quaternion[1]
      ur5_pose_4.orientation.z = quaternion[2]
      ur5_pose_4.orientation.w = quaternion[3]
      while True:
          ur5 = UR5()
          #print "============ Press `Enter` to execute a movement using a pose goal ..."
          #raw_input()
          ur5.go_to_pose_goal(ur5_pose_4)
          ur5.trajectory_execution_status(ur5_pose_4)
          time.sleep(1)
          ur5.gripper_motion(1)

          ur5.go_to_pose_goal(ur5_pose_3)
          ur5.trajectory_execution_status(ur5_pose_3)
          #time.sleep(1)
          #ur5.gripper_motion(0)

          ur5.go_to_pose_goal(ur5_pose_5)
          ur5.trajectory_execution_status(ur5_pose_5)
          #time.sleep(1)
          #ur5.gripper_motion(2)

          ur5.go_to_pose_goal(ur5_pose_2)
          ur5.trajectory_execution_status(ur5_pose_2)
          #time.sleep(1)
          #ur5.gripper_motion(2)

          ur5.go_to_pose_goal(ur5_pose_0)
          ur5.trajectory_execution_status(ur5_pose_0)
          #time.sleep(1)

          ch = raw_input("Do you want to give more inputs (y/n): ")
          if ch == 'n':
              break

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
