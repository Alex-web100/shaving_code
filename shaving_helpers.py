#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class ShavingHelperNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate=2.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states

    def move_arm_back(self):
        rospy.loginfo('Moving Arm Back')
        pose = {'gripper_aperture':.005,'wrist_extension': 0, 'joint_wrist_pitch':0, 'joint_wrist_yaw':2.5, 'joint_head_pan':-1.6, 'joint_head_tilt':-.5,'joint_lift':.58}
        self.move_to_pose(pose)
        rospy.sleep(3)

    def move_to_shaving_position(self):
        rospy.loginfo('Moving to Shaving Position')
        pose = {'wrist_extension': .3, 'joint_wrist_pitch':-.3708, 'joint_wrist_yaw':.157, 'joint_lift':.8056, 'joint_gripper_finger_left':.1, 'joint_lift':.802}
        self.move_to_pose(pose)
        rospy.sleep(3)

    def arm_test_pos(self):
        rospy.loginfo('Moving to Arm Test Position')
        pose = {'wrist_extension':.2, 'joint_lift':.63}
        self.move_to_pose(pose)
        rospy.sleep(3)

    def arm_down_pos(self):
        rospy.loginfo('Moving to Arm Test Position')
        pose = {'joint_lift':.53}
        self.move_to_pose(pose)
        rospy.sleep(3)
    
    def grip(self):
        rospy.loginfo('Grip')
        pose = {'gripper_aperture':.005}
        self.move_to_pose(pose)
        rospy.sleep(1)

    def trigger_move_arm_back(self, request):

        self.move_arm_back()

        return TriggerResponse(
            success=True,
            message= 'Moved arm back!'
        )

    def trigger_shaving_position(self, request):
        self.move_to_shaving_position()

        return TriggerResponse(
            success=True,
            message= 'To Shaving Position!'
        )

    def trigger_arm_pos(self, request):
        self.arm_test_pos()

        return TriggerResponse(
            success=True,
            message='Moved to arm pos!'
        )

    def trigger_arm_pos_2(self, request):
        self.arm_down_pos()

        return TriggerResponse(
            success=True,
            message='Moved to arm pos!'
        )

    def trigger_grip(self, request):
        self.grip()

        return TriggerResponse(
            success=True,
            message = 'Grip complete'
        )

    def main(self):
        hm.HelloNode.main(self, 'shaving_helper', 'shaving_helper', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)


        self.trigger_write_hello_service = rospy.Service('move_arm_back',
                                                         Trigger,
                                                         self.trigger_move_arm_back)

        self.trigger_write_hello_service = rospy.Service('shaving_position',
                                                         Trigger,
                                                         self.trigger_shaving_position)

        self.trigger_write_hello_service = rospy.Service('arm_pos',
                                                         Trigger,
                                                         self.trigger_arm_pos)

        self.trigger_write_hello_service = rospy.Service('arm_pos_2',
                                                         Trigger,
                                                         self.trigger_arm_pos_2)

        self.trigger_write_hello_service = rospy.Service('grip',
                                                         Trigger,
                                                         self.trigger_grip)


        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        try:
            rospy.init_node('shaving_helper')

            listener=tf.TransformListener()
            node = ShavingHelperNode()
            node.main()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('tf Exception')
    except KeyboardInterrupt:
        rospy.loginfo('Process Closing')