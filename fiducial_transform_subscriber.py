#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import tf
from fiducial_msgs.msg import FiducialTransformArray

def handle_tag_pose(data):
    #Get firs FiducialTransform in the array
    """ msg = (data.transforms)[0]
    rospy.loginfo(msg)
 """
    try:
        listener.waitForTransform('fid50','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
        (trans,rot)=listener.lookupTransform('fid50','sel_point',rospy.Time(0.0))
        rospy.loginfo(trans)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo("tf exception")

if __name__=='__main__':
    rospy.init_node("fiducial_subscriber")

    listener=tf.TransformListener()

    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, handle_tag_pose)
    rospy.spin()