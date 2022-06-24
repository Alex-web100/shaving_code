#!/usr/bin/env python

import rospy
import tf
from datetime import date
import pickle
import tf2_msgs.msg
import geometry_msgs.msg
import open3d as o3d

class PtAdder:

    def __init__(self,data):
        self.pub_tf = rospy.Publisher("/tf",tf2_msgs.msg.TFMessage, queue_size=1)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id="camera_color_optical_frame"
            t.child_frame_id="sel_point"
            t.header.stamp=rospy.Time.now()
            t.transform.translation.x=data[0]
            t.transform.translation.y=data[1]
            t.transform.translation.z=data[2]
            
            t.transform.rotation.x=0
            t.transform.rotation.y=0
            t.transform.rotation.z=0
            t.transform.rotation.w=1

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    try:
        try:
            rospy.init_node('add_selected_point')
            listener=tf.TransformListener()
            today=str(date.today())
            
            pcd = o3d.io.read_point_cloud("/home/sashawald/Documents/shaving_code-main/clouds/"+today+".ply")
            pcd = pcd.voxel_down_sample(voxel_size=0.015)
            vis = o3d.visualization.VisualizerWithEditing()
            vis.create_window()
            vis.add_geometry(pcd)
            vis.run()  
            vis.destroy_window()
            pdata = vis.get_picked_points()

            (x,y,z)=pcd.points[pdata[0]]
            (x,y,z)=(-x,-y,-z)

            temp=PtAdder([x,y,z])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf error")
    except KeyboardInterrupt:
        rospy.loginfo("closing")

