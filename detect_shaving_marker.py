#!/usr/bin/env python

import rospy
import tf
import tf2_ros
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

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states

    def move_to_tag_location(self,trans,rot):
        rospy.loginfo('Moving to Tag Height')
        with self.joint_states_lock:
            i = self.joint_states.name.index('joint_lift')
            j = self.joint_states.name.index('wrist_extension')
            current_pos_lift = self.joint_states.position[i]
            current_pos_wrist = self.joint_states.position[j]
        pose = {'joint_lift': current_pos_lift + (trans)[2],
                'wrist_extension': current_pos_wrist + (trans)[1]}
        rospy.loginfo('current lift position '+str(current_pos_lift))
        rospy.loginfo('move lift by '+str((trans)[2]))
        rospy.loginfo('current wrist position '+str(current_pos_wrist))
        rospy.loginfo('move wrist by '+str((trans)[1]))
        self.move_to_pose(pose)
        rospy.sleep(1.0)
        
    def find_next_tag(self, tagname):
        rospy.loginfo('Moving to the Next Tag')

        base_move_alpha=0.1
        total=base_move_alpha
        next_tag_found = False

        while not next_tag_found:
            pose = {'translate_mobile_base':base_move_alpha}
            self.move_to_pose(pose)
            rospy.sleep(10)
            looktime = rospy.Time.now()
            rospy.sleep(1)

            try:
                listener.waitForTransform('link_aruco_top_wrist',tagname,looktime,rospy.Duration(1.0))
                (t_new,r_new)=listener.lookupTransform('link_aruco_top_wrist',tagname,looktime)
                #To IMPLEMENT: break only if the x distance between wrist/tag is within a certain threshold (e.g, near enough to reach the tag)
                print("--Wrist Transformation to Newly Detected Tag--")
                print(t_new,r_new)
                self.trans=t_new
                self.rot=r_new
                if abs((self.trans)[0])<=.04:
                    print("next tag found)")
                    next_tag_found = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                continue
            print("Total: "+str(total))
            if total>.4:
                print("test")
                exit()
            total=total+base_move_alpha
            
    def move_arm_back(self):
        rospy.loginfo('Moving Arm Back')
        pose = {'wrist_extension': 0, 'joint_wrist_pitch':0, 'joint_head_pan':-1.6, 'joint_head_tilt':-.5}
        self.move_to_pose(pose)
        rospy.sleep(1.0)


    def trigger_move_to_height(self, request):
        
        listener.waitForTransform('link_aruco_top_wrist','shaver_aruco_test',rospy.Time(0.0),rospy.Duration(1.0))
        (self.trans,self.rot)=listener.lookupTransform('link_aruco_top_wrist','shaver_aruco_test',rospy.Time(0.0))
        print("--Wrist Transformation--")
        print(self.trans,self.rot)

        self.move_to_tag_location(self.trans,self.rot)
        self.find_next_tag('shaver_aruco_test_2')
        self.move_to_tag_location(self.trans,self.rot)

        return TriggerResponse(
            success=True,
            message='Completed Movement :) Remember to restart the detect_shaving_marker_node!'
        )

    def trigger_move_arm_back(self, request):

        self.move_arm_back()

        return TriggerResponse(
            success=True,
            message= 'Moved arm back!'
        )

    def main(self):
        hm.HelloNode.main(self, 'detect_shaving_marker', 'detect_shaving_marker', wait_for_first_pointcloud = False)

        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        
        self.trigger_write_hello_service = rospy.Service('/hello_world/trigger_move_to_height',
                                                         Trigger,
                                                         self.trigger_move_to_height)

        self.trigger_write_hello_service = rospy.Service('/hello_world/trigger_move_arm_back',
                                                         Trigger,
                                                         self.trigger_move_arm_back)

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
            node = MoveToTagNode()
            node.main()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('tf Exception')
    except KeyboardInterrupt:
        rospy.loginfo('Process Closing')