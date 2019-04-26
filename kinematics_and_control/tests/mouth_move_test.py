#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import TransformStamped,Vector3,Quaternion
from motion_executor import motion_executor
import numpy as np
import tf2_ros

rospy.init_node('mouth_move_test',anonymous=True)
m = motion_executor()
rospy.sleep(1.0)
broadcaster = tf2_ros.StaticTransformBroadcaster()
tfs = []

m.go_to_start()
m.go_waypoint("bin_A")
rospy.sleep(2.0)

m_tf = m.go_waypoint_mouth("bin_A")

static_transformStamped = TransformStamped()
static_transformStamped.header.stamp = rospy.Time.now()
static_transformStamped.header.frame_id = "/bin_A"
static_transformStamped.child_frame_id = "m_tf"
print(m_tf)
static_transformStamped.transform.translation = m_tf.pose.position
static_transformStamped.transform.rotation = m_tf.pose.orientation
tfs.append(static_transformStamped);
broadcaster.sendTransform(tfs)
print("tf pose broadcast")
rospy.sleep(6.0)


