#!/usr/bin/env python

import rospy
import tf
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

#Based on Kavya's stretch-bedding-manipulation code for detecting aruco markers, as well as the
#stretch_demo code, specifically the 'hello_world' node
class MoveToTagNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate=2.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.trans = trans

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states

    def move_to_tag_height(self):
        rospy.loginfo('Moving to Tag Height')
        with self.joint_states_lock:
            i = self.joint_states.name.index('joint_lift')
            j = self.joint_states.name.index('wrist_extension')
            current_pos_lift = self.joint_states.position[i]
            current_pos_wrist = self.joint_states.position[j]
        pose = {'joint_lift': current_pos_lift + (self.trans)[2],
                'wrist_extension': current_pos_wrist + (self.trans)[1]}
        rospy.loginfo('current lift position '+str(current_pos_lift))
        rospy.loginfo('move lift by '+str((self.trans)[2]))
        rospy.loginfo('current wrist position '+str(current_pos_wrist))
        rospy.loginfo('move wrist by '+str((self.trans)[1]))
        self.move_to_pose(pose)
        rospy.sleep(1.0)

    def trigger_move_to_height(self, request):
        
        self.move_to_tag_height()

        return TriggerResponse(
            success=True,
            message='Completed Movement :) Remember to restart the detect_shaving_marker_node!'
        )

    def main(self):
        hm.HelloNode.main(self, 'detect_shaving_marker', 'detect_shaving_marker', wait_for_first_pointcloud = False)


        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        
        self.trigger_write_hello_service = rospy.Service('/hello_world/trigger_move_to_height',
                                                         Trigger,
                                                         self.trigger_move_to_height)

        """ rospy.wait_for_service('/funmap/trigger_reach_until_contact')
        rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_reach_until_contact.')
        self.trigger_reach_until_contact_service = rospy.ServiceProxy('/funmap/trigger_reach_until_contact', Trigger) """
 
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        try:
            rospy.init_node('detect_shaving_marker')
            listener=tf.TransformListener()

            listener.waitForTransform('link_aruco_top_wrist','shaver_aruco_test',rospy.Time(0.0),rospy.Duration(1.0))
            (trans,rot)=listener.lookupTransform('link_aruco_top_wrist','shaver_aruco_test',rospy.Time(0.0))
            print("--Wrist Transformation--")
            print(trans,rot)

            node = MoveToTagNode()
            node.main()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('tf Exception')
    except KeyboardInterrupt:
        rospy.loginfo('Process Closing')
