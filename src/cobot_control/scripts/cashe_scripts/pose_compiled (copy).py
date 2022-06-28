#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
from dh_gripper_msgs.msg import GripperCtrl




class UR5:

  def __init__(self):
    #super(subscriber_class, self).__init__()

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

  def go_to_pose_goal(self):

    move_group = self.move_group
    global pose_goal, count
    pose_goal = Pose()

    pose_goal.position.x = input('x coordinate: ')
    pose_goal.position.y = input('y coordinate: ')
    pose_goal.position.z = input('z coordinate: ')
    pose_goal.orientation.w = input('w coordinate (orientation): ')
    rospy.loginfo(pose_goal)
    #pub_pose.publish(pose_goal)
    #flag_plan = self._group.go(wait=True)

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    if (plan == True):
        print "Got it"
    else:
        print "lmao"
    move_group.stop()
    move_group.clear_pose_targets()


def main():
  try:
      while True:
          ur5 = UR5()
          print "============ Press `Enter` to execute a movement using a pose goal ..."
          raw_input()
          ur5.go_to_pose_goal()
          ch = raw_input("Do you want to give more inputs (y/n): ")
          if ch == 'n':
              break

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
