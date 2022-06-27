#!/usr/bin/env python
import queue
import rospy
import tf
import numpy as np
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

class ExecuteTrajectoryNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate=2.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states

    def trajectory_callback(self, points_list):
        self.data = points_list.data
        self.points = np.reshape(self.data,(len(self.data)/3,3))

    def trigger_trajec_execution(self, request):
        self.points = np.flip((self.points[self.points[:,0].argsort()]),0)
        rospy.loginfo("List of points: "+str(self.points))

        prev=self.points[0]
        moves=[]
        for pt in self.points:
            [xdiff,ydiff,zdiff] = pt-prev
            moves.append([xdiff,ydiff,zdiff])
            prev=pt
        rospy.loginfo(moves)
        for move in moves:
            with self.joint_states_lock:
                i = self.joint_states.name.index('joint_lift')
                j = self.joint_states.name.index('wrist_extension')
                current_pos_lift = self.joint_states.position[i]
                current_pos_wrist = self.joint_states.position[j]
            pose={'translate_mobile_base':xdiff}
            self.move_to_pose(pose)
            rospy.sleep(2.0)
            pose={'wrist_extension':current_pos_wrist+ydiff}
            self.move_to_pose(pose)
            rospy.sleep(2.0)
            pose={'joint_lift':current_pos_lift+zdiff}
            self.move_to_pose(pose)
            rospy.sleep(1.0)
        return TriggerResponse(
            success=True,
            message='Executed Trajectory'
        )

    def main(self):
        hm.HelloNode.main(self, 'exec_trajectory', 'exec_trajectory', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        self.trajectory_subscriber = rospy.Subscriber('/trajectory_points',Float64MultiArray, self.trajectory_callback)

        self.trigger_write_hello_service = rospy.Service('execute_trajectory',
                                                         Trigger,
                                                         self.trigger_trajec_execution)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__=='__main__':
    rospy.init_node("exec_trajectory")
    
    node = ExecuteTrajectoryNode()
    node.main()