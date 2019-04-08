#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import Transform,Vector3,Quaternion
from scooper_duper.msg import *
from waypoint_lookup import get_waypoint_pose as get_waypoint_pose
# because of transformations
import tf
import math
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
    """
    Planning scene doesn't update immeadiatly so when adding boxes and tings need to wait for the scene to update
    If box isn't appearing likely to be due to it's parent transform not being published
    """
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
	    #Move it commander initialisation
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        #UR10's planning group name
        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        #Set max speed (only applies on joint moves)
	    self.group.set_max_velocity_scaling_factor(0.06)
        #self.group.set_max_velocity_scaling_factor(0.2)
        #publishes planned path in rviz
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        
        self.planning_frame = self.group.get_planning_frame()
        #Frame we are planning relative to (world frame) WHICH is not the same as the robot's base frame
        #The positions shown on the robot's pendant are in the tool0 frame which is offset via the TCP setting when initialising the robot
        #print(self.planning_frame)

        #publishers for the shelf frame and gripper sucker frame
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster2 = tf2_ros.StaticTransformBroadcaster()
        #Listener for various transforms allows us to convert between various frames
        self.transformer = tf.TransformListener(True,rospy.Duration(10.0))#self.tf_buffer)
        #Build the shelfs at (x,y,z) (in world frame)
        #self.build_scene((0,-1.077-0.34,0.879))
        self.clear_plan()
        
    def go_into_bin(self):
        current_pose = self.group.get_current_pose().pose()
    def go_to_start(self):
        starting_angles =[-98*math.pi/180,-80*math.pi/180,-107*math.pi/180,-82*math.pi/180,90*math.pi/180,8*math.pi/180]
        self.go_to_joint_config(starting_angles)
        #pose = 
        #print(current_pose_stamped)
    def build_scene(self,shelf_pos):
        
        #publish shelf frame that is static, later make variable for calibration
        
        shelf_transform = geometry_msgs.msg.TransformStamped()

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
        
        self.broadcaster.sendTransform(shelf_transform)
        
        #sucker position relative to the end effector link
        gripper_transform = geometry_msgs.msg.TransformStamped()
        gripper_transform.header.stamp = rospy.Time.now()
        gripper_transform.header.frame_id = "ee_link"
        gripper_transform.child_frame_id = "gripper_sucker"

        gripper_transform.transform.translation.x = float(0)
        gripper_transform.transform.translation.y = float(0)
        gripper_transform.transform.translation.z = float(0.34)

        quat = tf.transformations.quaternion_from_euler(
                   float(0),float(0),float(0))
        gripper_transform.transform.rotation.x = quat[0]
        gripper_transform.transform.rotation.y = quat[1]
        gripper_transform.transform.rotation.z = quat[2]
        gripper_transform.transform.rotation.w = quat[3]
        self.broadcaster2.sendTransform(gripper_transform)
        #Wait for transforms to be published
        rospy.sleep(2)
        #Making shelving out of planning boxes 
        success = True
        #Pose of each shelf
        shelf_pose = geometry_msgs.msg.PoseStamped()
        shelf_pose.header.frame_id = "shelves"
        #building the middle shelves
        for i in range(5):
            shelf_pose.pose.position.z = -i*0.32
            shelf_pose.pose.orientation.w = 1
            shelf_name = "mid_shelf"+str(i)
            self.scene.add_box(shelf_name, shelf_pose, size=(0.9, 0.6, 0.075))
            #if successful in building shelves every planning box will of been added to the scene
            success = success and wait_for_state_update(self.scene,box_name = shelf_name,box_is_known=True,timeout=2)
            
        #Side of shelving
        side_pose = geometry_msgs.msg.PoseStamped()
        side_pose.header.frame_id = "shelves"
        for i in (-1,1):
            side_pose.pose.position.y = 0
            side_pose.pose.position.z = -2.5*0.32
            side_pose.pose.position.x = i*0.86/2
            side_pose.pose.orientation.w = 1
            shelf_name = "side_shelf"+str(i+1)
            self.scene.add_box(shelf_name, side_pose, size=(0.04, 0.6, 1.75))
            success = success and wait_for_state_update(self.scene,box_name = shelf_name,box_is_known=True,timeout=2)


        print("shelves built = " + str(success))
        
    def clear_scene(self):
        #TODO
        print("remove box")    
    def execute_plan(self):
        #excute linear motion without blocking    
        self.group.execute(self.plan, wait=False)
    def print_pose(self):
        #prints the current pose of the end effector link relative to various frames
        current_pose_world = self.group.get_current_pose()
        print("Current pose in the world:")
        print(current_pose_world)
        pose_quat = current_pose_world.pose.orientation
        pose_quat = (pose_quat.x,pose_quat.y,pose_quat.z,pose_quat.w);
        angles = tf.transformations.euler_from_quaternion(pose_quat)
        print("Angles in the world:" +str(angles))
        
        current_pose_EE = self.transformer.transformPose("/base",current_pose_world)
        print("Current pose in the base frame:")
        print(current_pose_EE)
        pose_quat = current_pose_EE.pose.orientation
        pose_quat = (pose_quat.x,pose_quat.y,pose_quat.z,pose_quat.w);
        angles = tf.transformations.euler_from_quaternion(pose_quat)
        print("Angles in the base:" +str(angles))
        #self.tf_buffer.lookup_transform("/tool0","/world",rospy.Time(),rospy.Duration(1.0))
        c_pose = self.group.get_current_pose()
        c_pose.header.stamp = rospy.Time()
        #tool relative to the endeffector will have zero translation
        #TODO need to get the tool0 frame relative to base
        self.transformer.waitForTransform("/tool0","/world",rospy.Time(),rospy.Duration(2.0))
        current_pose_tool = self.transformer.transformPose("/tool0",c_pose)
        print("Current pose in the tool frame:")
        print(current_pose_tool)
        pose_quat = current_pose_tool.pose.orientation
        pose_quat = (pose_quat.x,pose_quat.y,pose_quat.z,pose_quat.w);
        angles = tf.transformations.euler_from_quaternion(pose_quat)
        print("Angles in the tool:" +str(angles))
 

    #def move_into_box(self,start_pose,dir):
    def check_complete(self):
        #Checks if current pose is cose to target position (within two cm)
        #TODO should be really if last joint traj has been reached
        current_pose_stamped = self.group.get_current_pose()
        current_pose = self.transformer.transformPose("/world",current_pose_stamped).pose
        #print(self.goal_pose)
        #print(current_pose)
        tolerance = 0.02
        x_close = abs(self.goal_pose.position.x - current_pose.position.x) < tolerance
        y_close = abs(self.goal_pose.position.y - current_pose.position.y) < tolerance
        z_close = abs(self.goal_pose.position.z - current_pose.position.z) < tolerance
        return (x_close and y_close and z_close) #all_close(self.goal_pose.position, current_pose.position,0.03)

    def go_to_joint_config(self,joint_goal):
        #Performs a blocking joint move
        self.group.go(joint_goal, wait=True)
        self.group.stop()
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def clear_plan(self):
         self.waypoints = []
    def add_plan_pose(self,pose):
        #Appends pose to the the plan, pose should be relative to the world frame
        self.waypoints.append(copy.deepcopy(pose))
    def compute_plan(self):
        print(self.waypoints)
        #Produce straight line move plan
        (plan, fraction) = self.group.compute_cartesian_path(
                                           self.waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0)         # jump_threshold
        #Re scale trajectory velocity, typical value 0.05
        plan = self.group.retime_trajectory(self.robot.get_current_state(),plan, 0.05);
        return plan
    def go_pose(self,pose_stamped):
	#print(self.transformer.getFrameStrings())
        #get pose in world frame
        self.transformer.waitForTransform("/world","/base", rospy.Time(),rospy.Duration(2.0))
        pose_stamped = self.transformer.transformPose("/world",pose_stamped)
        
        EE_goal_pose = pose_stamped.pose
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "/world"
        static_transformStamped.child_frame_id = "EE_goal_pose"

        static_transformStamped.transform.translation = EE_goal_pose.position
        static_transformStamped.transform.rotation = EE_goal_pose.orientation
        #publish Goal pose
        broadcaster.sendTransform(static_transformStamped)
        #stop current movement
        self.group.stop()
        #clear the plan
        self.clear_plan()
        #wpose = self.group.get_current_pose()
        
        #Move it's Straight line trajectory can go between multiple points, however all moves are implimented point to point moves
        #So only add one pose
        self.add_plan_pose(EE_goal_pose)
        
        
        #compute the trajectory
        self.plan = self.compute_plan()
        
        #print("---------------------")
        #print(len(self.plan.joint_trajectory.points))
        #print("---------------------")
        #Variables used to compute how close we are to the 
        self.goal_pose = EE_goal_pose
        self.finished_move = False
        #start the motion
        self.execute_plan()

    def go_waypoint(self,waypoint_id):
        #looks up waypoint and then perfroms straight line to it
        self.go_pose(get_waypoint_pose(waypoint_id))

if __name__ == '__main__':
    try:
        rospy.init_node('move_group_python_interface_tutorial',
                       anonymous=True)
        m = motion_executor()
        m.go_to_start()
        rospy.sleep(1)        
        m.go_waypoint("tote")
        rospy.sleep(8)
        m.go_waypoint("bin_A")
       # m.go_into_bin()
        #while rospy.:
           # print(m.check_complete())
            #rospy.sleep(1)
        #rospy.sleep(8)
        #m.go_waypoint("bin_B")
        #rospy.sleep(8)
        #m.go_waypoint("bin_A")
        #rospy.sleep(5)
        #m.go_waypoint("tote")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
