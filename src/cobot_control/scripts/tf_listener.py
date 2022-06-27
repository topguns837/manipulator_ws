#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('base', 'tool0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rospy.loginfo("X co-ordinate is %s", trans[0])
        rospy.loginfo("Y co-ordinate is %s", trans[1])
        rospy.loginfo("Z co-ordinate is %s", trans[2])
        rospy.loginfo("Rx is %s", rot[0])
        rospy.loginfo("Ry is %s", rot[1])
        rospy.loginfo("Rz is %s", rot[2])
        rate.sleep()
