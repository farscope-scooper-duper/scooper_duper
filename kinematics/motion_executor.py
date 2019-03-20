#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import Transform,Vector3,Quaternion
from scooper_duper.msg import *
from waypoint_lookup import get_waypoint_pose as get_waypoint_pose
# because of transformations
import tf

import tf2_ros
#import sys
#import copy
#import rospy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import sys
import copy

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True
def wait_for_state_update(scene,box_name = "", box_is_known=False, box_is_attached=False, timeout=4):

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

class motion_executor():
    def __init__(self):
        #super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)

        self.robot = moveit_commander.RobotCommander()


        self.scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_max_velocity_scaling_factor(0.2)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.planning_frame = self.group.get_planning_frame()
        print(self.planning_frame)
        self.build_scene((0,-1.3,3*0.32))
        self.clear_plan()
    def build_scene(self,shelf_pos):
        
        #publish shelf frame that is static, later make variable for calibration
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "shelves"

        static_transformStamped.transform.translation.x = float(shelf_pos[0])
        static_transformStamped.transform.translation.y = float(shelf_pos[1])
        static_transformStamped.transform.translation.z = float(shelf_pos[2])

        quat = tf.transformations.quaternion_from_euler(
                   float(0),float(0),float(0))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
        rospy.sleep(2)
        success = True
        shelf_pose = geometry_msgs.msg.PoseStamped()
        shelf_pose.header.frame_id = "shelves"
        for i in range(5):
            shelf_pose.pose.position.z = -i*0.32
            shelf_pose.pose.orientation.w = 1
            shelf_name = "mid_shelf"+str(i)
            self.scene.add_box(shelf_name, shelf_pose, size=(0.9, 0.6, 0.075))
            success = success and wait_for_state_update(self.scene,box_name = shelf_name,box_is_known=True,timeout=10)
        side_pose = geometry_msgs.msg.PoseStamped()
        side_pose.header.frame_id = "shelves"
        for i in (-1,1):
            side_pose.pose.position.y = 0
            side_pose.pose.position.z = -2.5*0.32
            side_pose.pose.position.x = i*0.86/2
            side_pose.pose.orientation.w = 1
            shelf_name = "side_shelf"+str(i+1)
            self.scene.add_box(shelf_name, side_pose, size=(0.04, 0.6, 1.75))
            success = success and wait_for_state_update(self.scene,box_name = shelf_name,box_is_known=True,timeout=10)


        print("shelves built = " + str(success))
        print(success)
    def clear_scene(self):
        print("remove box")    
    def execute_plan(self):
        self.group.execute(self.plan, wait=False)

    #def move_into_box(self,start_pose,dir):

    def go_to_joint_config(self,joint_goal):

        group.go(joint_goal, wait=True)
        group.stop()
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def clear_plan(self):
         self.waypoints = []
    def add_plan_pose(self,pose):
        self.waypoints.append(copy.deepcopy(pose))
    def compute_plan(self):
        print(self.waypoints)
        (plan, fraction) = self.group.compute_cartesian_path(
                                           self.waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0)         # jump_threshold
        return plan
    def go_pose(self,pose):

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "EE_pose"

        static_transformStamped.transform.translation = pose.position
        static_transformStamped.transform.rotation = pose.orientation

        broadcaster.sendTransform(static_transformStamped)


        self.clear_plan()
        wpose = self.group.get_current_pose()
        print wpose
        self.add_plan_pose(pose)
        self.plan = self.compute_plan()
        self.finished_move = False
        self.error = False
        self.execute_plan()

    def go_waypoint(self,waypoint_id):
        
        self.go_pose(get_waypoint_pose(waypoint_id))

if __name__ == '__main__':
    try:
        m = motion_executor()
        rospy.sleep(1)        
        m.go_waypoint("tote")
        rospy.sleep(4)
        m.go_waypoint("bin_A")
        rospy.sleep(4)
        m.go_waypoint("bin_B")
        rospy.sleep(4)
        m.go_waypoint("bin_E")
        #rospy.sleep(5)
        #m.go_waypoint("tote")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
