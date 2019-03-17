#!/usr/bin/env python  

from kinematics_lib import *
#import ur_kinematics as kin
import numpy as np
import tf
import roslib
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler,euler_from_quaternion

class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub2 = rospy.Publisher("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)
        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(1)
                 
            #print(mat);
            #q = invKine(mat);
            
           

            theta1 = np.radians(0.0)
            theta2 = np.radians(0.0)
            theta3 = np.radians(0.0)
            theta4 = np.radians(0.0)
            theta5 = np.radians(0.0)
            theta6 = np.radians(0.0)

            th_origin = np.matrix([[theta1], [theta2], [theta3], [theta4], [theta5], [theta6]])
            c = [0]
            location = HTrans(th_origin,c )
            print("----FK-----"); 
            print(location)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "world"

            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "target_end_pose"

            trans = tf.transformations.translation_from_matrix(location);
            print(trans)
            rot = tf.transformations.quaternion_from_matrix(location);
            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]
            t.transform.rotation.x = rot[0]
            t.transform.rotation.y = rot[1]
            t.transform.rotation.z = rot[2]
            t.transform.rotation.w = rot[3]
            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)


          #  print(location)
           # print("before")

            q = invKine(location)

            #print("IK")
            #print(th)
            ##print(q)
            print("----Orginal angles---");
            print(th_origin)
            print("----IK angles--"); 
            for k in range(8):
                angles = q[:,k].transpose()
                angles = angles.tolist()[0]
                print(angles);

            sol_num = 1
            ik_pose = HTrans( q[:,sol_num],c )
            ik_angles = q[:,sol_num].transpose()
            ik_angles = ik_angles.tolist()[0]
                
            th_origin = th_origin.transpose()
            th_origin = th_origin.tolist()[0];
            
            print("--IK Position--"); 
            print(ik_pose);
            # Move between the orginal config, and new config should be same
            traj = JointTrajectory()
            traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

            now = rospy.get_rostime()
            rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
            traj.header.stamp = now
            p1 = JointTrajectoryPoint()
            p1.positions = th_origin
            p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            p1.time_from_start = rospy.Duration(5.0)
            traj.points.append(p1)
           # print(angles)
            p2 = JointTrajectoryPoint()
            p2.positions = ik_angles
            p2.velocities =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            p2.time_from_start = rospy.Duration(20.0)
            traj.points.append(p2)

            ag = FollowJointTrajectoryActionGoal()
            ag.goal.trajectory = traj
            self.pub2.publish(ag)
            rospy.sleep(22)
if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()







