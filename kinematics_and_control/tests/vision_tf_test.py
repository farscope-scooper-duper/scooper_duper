#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import TransformStamped,Vector3,Quaternion
from motion_executor import motion_executor
import numpy as np
import tf2_ros
rospy.init_node('vision_tf_test',anonymous=True)
m = motion_executor()
rospy.sleep(1.0)
broadcaster = tf2_ros.StaticTransformBroadcaster()
tfs = []
for i in range(4):
    v_tf = m.go_vision_viewpoint(i,"bin_A")
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "/bin_A"
    static_transformStamped.child_frame_id = "v_tf"+str(i)
    print(v_tf)
    static_transformStamped.transform.translation = v_tf.pose.position
    static_transformStamped.transform.rotation = v_tf.pose.orientation
    tfs.append(static_transformStamped);
    rospy.sleep(6.0)
broadcaster.sendTransform(tfs)
print("tf pose broadcast")
