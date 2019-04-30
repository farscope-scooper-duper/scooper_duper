#!/usr/bin/env python
import rospy
import tf
import math
import tf2_ros
from geometry_msgs.msg import Transform,Vector3,Quaternion,TransformStamped
from waypoint_lookup import get_waypoint_pose as get_waypoint_pose
import shelf_config
        
class scooper_tf_publisher():

        def __init__(self):
            #publishers for the shelf frame and gripper sucker frame
            self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        def publish_tfs(self):
            shelf_pos = shelf_config.shelving_position
            tf_to_broadcast = []
            #camera frame relative to end effector link
            camera_transform = TransformStamped()
            camera_transform.header.stamp = rospy.Time.now()
            camera_transform.header.frame_id = "/ee_link"
            camera_transform.child_frame_id = "/camera"
            #Camera transform, GUESSED not calibrated
            camera_transform.transform.translation.x = float(-0.05)
            camera_transform.transform.translation.y = float(0)
            camera_transform.transform.translation.z = float(0.05)
            quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
            camera_transform.transform.rotation.x = quat[0]
            camera_transform.transform.rotation.y = quat[1]
            camera_transform.transform.rotation.z = quat[2]
            camera_transform.transform.rotation.w = quat[3]
            tf_to_broadcast.append(camera_transform)

            #gripper 'end' relative to end effector link
            gripper_transform = TransformStamped()
            gripper_transform.header.stamp = rospy.Time.now()
            gripper_transform.header.frame_id = "ee_link"
            gripper_transform.child_frame_id = "gripper_sucker"
            gripper_transform.transform.translation.x = float(0.08)
            gripper_transform.transform.translation.y = float(0)
            gripper_transform.transform.translation.z = float(0.6)
            quat = tf.transformations.quaternion_from_euler(
            float(0),float(0),float(0))
            gripper_transform.transform.rotation.x = quat[0]
            gripper_transform.transform.rotation.y = quat[1]
            gripper_transform.transform.rotation.z = quat[2]
            gripper_transform.transform.rotation.w = quat[3]
            tf_to_broadcast.append(gripper_transform)

            #front,top left corner of shelves
            shelf_transform = TransformStamped()
            shelf_transform.header.stamp = rospy.Time.now()
            shelf_transform.header.frame_id = "base"
            shelf_transform.child_frame_id = "shelves"
            shelf_transform.transform.translation.x = float(shelf_pos[0])
            shelf_transform.transform.translation.y = float(shelf_pos[1])
            shelf_transform.transform.translation.z = float(shelf_pos[2])
            quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
            shelf_transform.transform.rotation.x = quat[0]
            shelf_transform.transform.rotation.y = quat[1]
            shelf_transform.transform.rotation.z = quat[2]
            shelf_transform.transform.rotation.w = quat[3]
            tf_to_broadcast.append(shelf_transform)

            for bin_id in ["bin_A","bin_B","bin_C","bin_D","bin_E","bin_F","bin_G","bin_H","bin_I","bin_J","bin_K","bin_L"]:
                bin_pose = get_waypoint_pose(bin_id).pose
                bin_tf = TransformStamped()
                bin_tf.header.stamp = rospy.Time.now()
                bin_tf.header.frame_id = "/shelves"
                bin_tf.child_frame_id = "/" + bin_id

                bin_tf.transform.translation = bin_pose.position
                bin_tf.transform.rotation = bin_pose.orientation
                tf_to_broadcast.append(bin_tf)

            #tf2 specific bug where you can only use one broadcaster per process, so publish both transforms at once
            self.broadcaster.sendTransform(tf_to_broadcast)



if __name__ == '__main__':
    try:
        publisher = scooper_tf_publisher()
        rospy.init_node('scooper_tf_publisher', anonymous=True)
        rate = rospy.Rate(0.1) # 2hz
        
        while not rospy.is_shutdown():
            publisher.publish_tfs()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
