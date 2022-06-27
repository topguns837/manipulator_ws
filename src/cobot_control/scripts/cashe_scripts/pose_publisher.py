#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *

def talker():
    pub_pose = rospy.Publisher('publish_pose', Pose, queue_size=10)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pose_object = Pose()

        pose_object.position.x = input('x coordinate: ')
        pose_object.position.y = input('y coordinate: ')
        pose_object.position.z = input('z coordinate: ')
        pose_object.orientation.w = input('w coordinate (orientation): ')
        rospy.loginfo(pose_object)
        pub_pose.publish(pose_object)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python
import rospy
from dh_gripper_msgs.msg import GripperCtrl

def talker():
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    rospy.init_node('gripper', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = GripperCtrl()
    msg.initialize = False
    while True:

    	msg.position = input("Enter Position")
    	msg.force = input("Enter Force")
    	msg.speed = 100

        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
