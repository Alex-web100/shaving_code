#!/usr/bin/env python
import rospy
import tf
import numpy as np
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class AlignWithTagNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate=2.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states

    def ready(self,request):
        pose={'joint_head_pan':-1.5,'joint_head_tilt':-.45,'joint_lift':.82,'joint_wrist_pitch':0,'joint_wrist_roll':0,'joint_wrist_yaw':0}
        self.move_to_pose(pose)
        rospy.sleep(1)
        pose={'wrist_extension':.1}
        self.move_to_pose(pose)
        rospy.sleep(1)
        pose={'gripper_aperture':.075}
        self.move_to_pose(pose)
        rospy.sleep(5)
        pose={'gripper_aperture':-.1}
        self.move_to_pose(pose)
        rospy.sleep(1)
        return TriggerResponse(
            success=True,
            message='Completed Head Movement'
        )

    def trigger_move_to_tag(self,request):
        try:
            listener.waitForTransform('shaver_aruco_test','link_wrist_yaw_bottom',rospy.Time(0.0),rospy.Duration(1.0))
            (trans,rot)=listener.lookupTransform('shaver_aruco_test','link_wrist_yaw_bottom',rospy.Time(0.0))
            rospy.loginfo("tag to robot wrist: " + str((trans,rot)))

            listener.waitForTransform('overhead_shaver_aruco_test','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
            (trans2,rot2)=listener.lookupTransform('overhead_shaver_aruco_test','sel_point',rospy.Time(0.0))
            rospy.loginfo("tag to selected point: " + str((trans2,rot2)))

            self.finaltrans=np.subtract(trans,trans2)
            rospy.loginfo("robot to point:" + str(self.finaltrans))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")
        (x,y,z)=(self.finaltrans[0],self.finaltrans[1],self.finaltrans[2])
        with self.joint_states_lock:
            i = self.joint_states.name.index('joint_lift')
            j = self.joint_states.name.index('wrist_extension')
            current_pos_lift = self.joint_states.position[i]
            current_pos_wrist = self.joint_states.position[j]
        pose={'translate_mobile_base':x+.28}
        self.move_to_pose(pose)
        rospy.sleep(2.0)
        pose={'wrist_extension':current_pos_wrist+.03-y}
        self.move_to_pose(pose)
        rospy.sleep(2.0)
        pose={'joint_lift':current_pos_lift+.165-z}
        self.move_to_pose(pose)
        rospy.sleep(1.0)
        return TriggerResponse(
            success=True,
            message='Completed Alignment w/ Tag'
        )
    
    def main(self):
        hm.HelloNode.main(self, 'get_robot_trans_to_tag', 'get_robot_trans_to_tag', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        self.trigger_write_hello_service = rospy.Service('move_to_tag',
                                                         Trigger,
                                                         self.trigger_move_to_tag)
        self.trigger_write_hello_service = rospy.Service('ready',
                                                         Trigger,
                                                         self.ready)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__=='__main__':
    rospy.init_node("get_robot_trans_to_tag")

    listener=tf.TransformListener()
    
    node = AlignWithTagNode()
    node.main()