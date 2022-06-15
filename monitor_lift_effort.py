#!/usr/bin/env python

import rospy
import rosservice
import tf
import tf2_ros
import threading
import matplotlib.pyplot as plt
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import pickle

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from pid import PID

class LiftEffortNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.joint_states_lock = threading.Lock()
        self.joint_states=None
        self.time=[]
        self.val=[]

    def lift_pid(self):
        #initialize PD setpoint as first lift effort reported when the node is run
        lift = self.joint_states.name.index('joint_lift')
        test_pid=PID(self.joint_states.effort[lift],5,.7,.2)
        while not rospy.is_shutdown():
            curr_effort=self.joint_states.effort[lift]
            rospy.loginfo('Lift Effort: '+str(curr_effort))
            t=rospy.get_time()
            self.time.append(t)
            (new,change)=(test_pid.update(t,curr_effort))
            move_by=.001*change
            if .05<move_by<=.3 or -.2<=move_by<=-.05:
                pose={'joint_lift':self.joint_states.position[lift]+move_by}
                self.move_to_pose(pose)
                rospy.sleep(2)
                test_pid.setpoint=self.joint_states.effort[lift]
            self.val.append(new)
            print(str((new,change)))

    def plot(self,time,val):
        fig=plt.figure(figsize=[15,7])
        ax1=fig.add_axes([0,0,1,1])
        ax1.plot(time,val,color='green')
        print(time)
        print(val)

    def joint_states_callback(self,joint_states):
        with self.joint_states_lock:
            self.joint_states=joint_states

    def trigger_lift_pid(self, request):
        self.lift_pid()
        pickle.dump((self.time,self.val),open('/home/hello-robot/catkin_ws/src/stretch_ros/shaving_aruco/tempdata/obs.pkl','wb'))

        return TriggerResponse(
            success=True,
            message = 'Reported lift effort'
        ) 

    def main(self):
        hm.HelloNode.main(self, 'effort_node', 'effort_node', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        self.trigger_write_hello_service = rospy.Service('monitor_effort',Trigger,self.trigger_lift_pid)

        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node("effort_node")
        node = LiftEffortNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('Process Closing')