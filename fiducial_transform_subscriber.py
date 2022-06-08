#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import tf_conversions
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray

def handle_tag_pose(data):
    #Get firs FiducialTransform in the array
    print("TEST")
    msg = (data.transforms)[0]
    rospy.loginfo(msg)

if __name__=='__main__':
    rospy.init_node("fiducial_subscriber")
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, handle_tag_pose)
    rospy.spin()