#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]),''))
#sys.path.append("/home/julian/catkin_ws/src/scooper_duper")

for p in sys.path:
    print(p)
import rospy
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import Transform,Vector3,Quaternion
from scooper_duper.msg import *
from waypoint_lookup import get_waypoint_pose as get_waypoint_pose
import shelf_config
import numpy as np
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

from constants import *


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
        self.goal_pose=geometry_msgs.msg.Pose()
        #publishers for the shelf frame and gripper sucker frame
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        #Listener for various transforms allows us to convert between various frames
        self.transformer = tf.TransformListener(True,rospy.Duration(10.0))#self.tf_buffer)
        self.grip_box_name = "gripper_box"
        #Build the shelfs at (x,y,z) (in world frame)
        self.build_scene(shelf_config.shelving_position)
        self.clear_plan()
    
    def addgripper(self):
        # ----> (z towards shelves)
        # |
        # |
        #\|/ (x towards floor)
        #y
        grip_box_width = 0.040
        #x        
        grip_box_height = 0.030
        #z
        grip_box_length = 0.650
        gripper_box_pose = geometry_msgs.msg.PoseStamped()
        gripper_box_pose.header.frame_id = "ee_link"
        gripper_box_pose.pose.position.x = grip_box_height/2 
        gripper_box_pose.pose.position.y = 0
        gripper_box_pose.pose.position.z = grip_box_length/2+0.001
        
#        success = success and wait_for_state_update(self.scene,box_name =  grip_box_name,box_is_known=True,timeout=2)
        grasping_group = 'endeffector'
        self.touch_links = self.robot.get_link_names(group=grasping_group)
#        self.scene.attach_box(self.group.get_end_effector_link(),grip_box_name, touch_links=touch_links)
        #self.scene.remove_attached_object(self.group.get_end_effector_link(), name=grip_box_name)
        #wait_for_state_update(self.scene,box_name = grip_box_name ,box_is_known=False,timeout=2)
        self.scene.remove_world_object(self.grip_box_name)
        wait_for_state_update(self.scene,box_name = self.grip_box_name ,box_is_known=False,timeout=2)
        self.scene.add_box(self.grip_box_name, gripper_box_pose, size=(grip_box_height,grip_box_width,grip_box_length))
        success = wait_for_state_update(self.scene,box_name = self.grip_box_name ,box_is_known=True,timeout=2)
        self.scene.attach_box(self.group.get_end_effector_link(),self.grip_box_name, touch_links=self.touch_links)
        success = success and wait_for_state_update(self.scene,box_name = self.grip_box_name ,box_is_known=False,box_is_attached = True,timeout=2)
        return success
    def removegripper(self):
        #wait_for_state_update(self.scene,box_name = grip_box_name ,box_is_known=False,timeout=2)
        attached = self.scene.get_attached_objects([self.grip_box_name])
        if (attached):
            self.scene.remove_attached_object(self.group.get_end_effector_link(), name=self.grip_box_name)
            wait_for_state_update(self.scene,box_name = self.grip_box_name ,box_is_known=True,timeout=2)
        self.scene.remove_world_object(self.grip_box_name)
        success = wait_for_state_update(self.scene,box_name = self.grip_box_name ,box_is_known=False,timeout=2)
        return success

    def shutdown(self):
        self.group.stop()

    def go_into_bin(self):
        current_pose = self.group.get_current_pose().pose()


    def go_to_start(self):
        starting_angles =[-98*math.pi/180,-70*math.pi/180,-107*math.pi/180,-82*math.pi/180,90*math.pi/180,8*math.pi/180]
        self.removegripper()    
        self.go_to_joint_config(starting_angles)
        self.addgripper()
        #pose = 
        #print(current_pose_stamped)


    def build_scene(self,shelf_pos):
        
        #publish shelf frame that is static, later make variable for calibration
        #sucker position relative to the end effector link
        tf_to_broadcast = []
        gripper_transform = geometry_msgs.msg.TransformStamped()
        gripper_transform.header.stamp = rospy.Time.now()
        gripper_transform.header.frame_id = "ee_link"
        gripper_transform.child_frame_id = "gripper_sucker"
        gripper_transform.transform.translation.x = float(0.08)
        gripper_transform.transform.translation.y = float(0)
        gripper_transform.transform.translation.z = float(0.625)
        quat = tf.transformations.quaternion_from_euler(
                   float(0),float(0),float(0))
        gripper_transform.transform.rotation.x = quat[0]
        gripper_transform.transform.rotation.y = quat[1]
        gripper_transform.transform.rotation.z = quat[2]
        gripper_transform.transform.rotation.w = quat[3]
        tf_to_broadcast.append(gripper_transform)
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
        tf_to_broadcast.append(shelf_transform)

        for bin_id in ["bin_A","bin_B","bin_C","bin_D","bin_E","bin_F","bin_G","bin_H","bin_I","bin_J","bin_K","bin_L"]:
            bin_pose = get_waypoint_pose(bin_id).pose
            bin_tf = geometry_msgs.msg.TransformStamped()
            bin_tf.header.stamp = rospy.Time.now()
            bin_tf.header.frame_id = "/shelves"
            bin_tf.child_frame_id = "/" + bin_id

            bin_tf.transform.translation = bin_pose.position
            bin_tf.transform.rotation = bin_pose.orientation
            tf_to_broadcast.append(bin_tf)




        #tf2 specific bug where you can only use one broadcaster per process, so publish both transforms at once
        self.broadcaster.sendTransform(tf_to_broadcast)
        self.transformer.waitForTransform("/shelves","/base",rospy.Time.now(),rospy.Duration(2.0))
        
        #Wait for transforms to be published
        rospy.sleep(2)
        #Making shelving out of planning boxes 
        success = True
        #Pose of each shelf
        shelf_pose = geometry_msgs.msg.PoseStamped()
        shelf_pose.header.frame_id = "shelves"

        #building the middle shelves
        for i in range(5):
            shelf_pose.pose.position.x = -shelf_config.shelf_width/2
            shelf_pose.pose.position.y = -shelf_config.shelf_depth/2
            shelf_pose.pose.position.z = -i*shelf_config.shelf_separation- shelf_config.shelf_height/2
            shelf_pose.pose.orientation.w = 1
            shelf_name = "mid_shelf"+str(i)
            self.scene.add_box(shelf_name, shelf_pose, size=(shelf_config.shelf_width, shelf_config.shelf_depth,shelf_config.shelf_height))
            #if successful in building shelves every planning box will of been added to the scene
            success = success and wait_for_state_update(self.scene,box_name = shelf_name,box_is_known=True,timeout=2)
            
        #Side of shelving
        side_pose = geometry_msgs.msg.PoseStamped()
        side_pose.header.frame_id = "shelves"

        for i in (0,1):
            side_pose.pose.position.x = -i*shelf_config.shelf_width + (i-0.5)*2*shelf_config.side_width/2
            side_pose.pose.position.y = -shelf_config.shelf_depth/2
            side_pose.pose.position.z = -shelf_config.side_height/2 #-2.5*0.32
            
            side_pose.pose.orientation.w = 1
            shelf_name = "side_shelf"+str(i+1)
            self.scene.add_box(shelf_name, side_pose, size=(shelf_config.side_width, shelf_config.shelf_depth, shelf_config.side_height))
            success = success and wait_for_state_update(self.scene,box_name = shelf_name,box_is_known=True,timeout=2)
        self.removegripper()
        success = success and self.addgripper()
        
        print("Planning scene built = " + str(success))
        

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
        position,pose_quat = self.transformer.lookupTransform("/tool0","/world",rospy.Time())
        print("Current position in the tool frame:")
        print(position)
        pose_quat = (pose_quat[0],pose_quat[1],pose_quat[2],pose_quat[3]);
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
        tolerance = 0.0075
        #tolerance = 0.009
        o_tolerance = 0.001
        #o_tolerance = 0.009
        x_close = abs(self.goal_pose.position.x - current_pose.position.x)# < tolerance
        y_close = abs(self.goal_pose.position.y - current_pose.position.y)# < tolerance
        z_close = abs(self.goal_pose.position.z - current_pose.position.z)# < tolerance
        #
        goal_orientation = [self.goal_pose.orientation.x, self.goal_pose.orientation.y, self.goal_pose.orientation.z, self.goal_pose.orientation.w]
        current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        o_x_close = abs(self.goal_pose.orientation.x - current_pose.orientation.x)
        o_y_close = abs(self.goal_pose.orientation.y - current_pose.orientation.y)
        o_z_close = abs(self.goal_pose.orientation.z - current_pose.orientation.z)
        o_w_close = abs(self.goal_pose.orientation.w - current_pose.orientation.w)

        position_error = np.sqrt(x_close**2+y_close**2+z_close**2)
        position_close = (position_error < tolerance)

        orientation_error = abs(np.dot(goal_orientation, current_orientation))
        orientation_close = (orientation_error > 1 - o_tolerance) 

        print("EE pose error position: "  + str(position_error) + "("+str(position_close)+")")
        print("EE pose error orientation: "+ str(orientation_error) + "("+str(orientation_close)+")")
                
        return position_close and orientation_close #(x_close and y_close and z_close) #all_close(self.goal_pose.position, current_pose.position,0.03)

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
        (plan, fraction) = self.group.compute_cartesian_path(self.waypoints, 0.01,0)         # jump_threshold
        #Re scale trajectory velocity, typical value 0.05
        plan = self.group.retime_trajectory(self.robot.get_current_state(),plan, SPEED_SCALE);
        return plan
    def wait_till_complete(self,waypoint_id=''):    
        while (not rospy.is_shutdown()):
            rospy.sleep(0.1) 
            if waypoint_id <> '':
             m.go_waypoint(waypoint_id)   
            if self.check_complete():
                break 
        return
        rospy.sleep(0.5) 
    def go_pose(self,pose_stamped):
        #get pose in world frame
        try: 
            #print("Mex Moving to pose:")
            #print(pose_stamped.header.frame_id)     
            now = rospy.Time.now()
            
            self.transformer.waitForTransform(pose_stamped.header.frame_id,"/world", now,rospy.Duration(20.0))

            pose_stamped = self.transformer.transformPose("/world",pose_stamped)
            
            self.goal_pose = pose_stamped.pose
            
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "/world"
            static_transformStamped.child_frame_id = "EE_goal_pose"

            static_transformStamped.transform.translation = self.goal_pose.position
            static_transformStamped.transform.rotation = self.goal_pose.orientation
            #publish Goal pose
            self.broadcaster.sendTransform(static_transformStamped)
            
            
            if not self.check_complete():
                #stop current movement
                self.group.stop()
                #Just a little pause
                rospy.sleep(0.1)
                #clear the plan
                self.clear_plan()
                #wpose = self.group.get_current_pose().pose
                #self.add_plan_pose(wpose)
                
                #Moveit's Straight line trajectory can go between multiple points, however all moves are implemented point to point moves
                #So only add one pose
                self.add_plan_pose(self.goal_pose)
                #compute the trajectory
                self.plan = self.compute_plan()
                self.robot
                #print("--- self.go_pose(viewpoint_pose)------------------")
                #print(len(self.plan.joint_trajectory.points))
                #print("---------------------")
                #Variables used to compute how close we are to the 
                #start the motion
                self.execute_plan()
        except Exception as e:            
            print(e)
    def go_relative_pose(self, position,orientation):
        relative_pose = geometry_msgs.msg.PoseStamped()
        relative_pose.header.stamp = rospy.Time.now()
        relative_pose.header.frame_id = "/ee_link"
        relative_pose.pose.position =  geometry_msgs.msg.Vector3(*position)
        #quat = tf.transformations.quaternion_from_euler(viewpoint_rotations[vision_id][0],viewpoint_rotations[vision_id][1],viewpoint_rotations[vision_id][2])
        relative_pose.pose.orientation =  geometry_msgs.msg.Quaternion(*orientation)
        self.go_pose(relative_pose)
    
    def go_vision_viewpoint(self,vision_id,bin_id):
        print("Going to viewpoint:" + str(vision_id))
        viewpoint_positions = ((0,-0.15,0),(0,0.15,0),(-0.15,0,0), (0,0,0))
        viewpoint_rotations = np.array(((-20,0,0),(20,0,0),(0,20,0),(0,0,0)))
        viewpoint_rotations = np.deg2rad(viewpoint_rotations)
        
        viewpoint_pose = geometry_msgs.msg.PoseStamped()
        viewpoint_pose.header.stamp = rospy.Time.now()
        viewpoint_pose.header.frame_id = "/" + bin_id
        viewpoint_pose.pose.position.x = float(viewpoint_positions[vision_id][0])
        viewpoint_pose.pose.position.y = float(viewpoint_positions[vision_id][1])
        viewpoint_pose.pose.position.z = float(viewpoint_positions[vision_id][2])
        quat = tf.transformations.quaternion_from_euler(viewpoint_rotations[vision_id][0],viewpoint_rotations[vision_id][1],viewpoint_rotations[vision_id][2])
        viewpoint_pose.pose.orientation.x = quat[0]
        viewpoint_pose.pose.orientation.y = quat[1]
        viewpoint_pose.pose.orientation.z = quat[2]
        viewpoint_pose.pose.orientation.w = quat[3]
        self.go_pose(viewpoint_pose)
        return viewpoint_pose
        
    def go_waypoint_mouth(self,bin_id): #Moves to the bin mouth corresponding to bin_id (15cm forward of bin_id's waypoint).
        mouth_offset = (0.0, 0.0, 0.09)
        mouth_pose = geometry_msgs.msg.PoseStamped()
        mouth_pose.header.stamp = rospy.Time.now()
        mouth_pose.header.frame_id = "/" + bin_id
        mouth_pose.pose.position.x = float(mouth_offset[0])
        mouth_pose.pose.position.y = float(mouth_offset[1])
        mouth_pose.pose.position.z = float(mouth_offset[2])
        mouth_pose.pose.orientation.x = 0
        mouth_pose.pose.orientation.y = 0
        mouth_pose.pose.orientation.z = 0
        mouth_pose.pose.orientation.w = 1
        self.go_pose(mouth_pose)
        return mouth_pose

    def go_waypoint(self,waypoint_id):
        print("going to waypoint: " + waypoint_id)
        #looks up waypoint and then perfroms straight line to it
        self.go_pose(get_waypoint_pose(waypoint_id))

if __name__ == '__main__':
    try:
        rospy.init_node('move_group_python_interface_tutorial',
                       anonymous=True)
        m = motion_executor()
        
        m.go_to_start()
        #m.wait_till_complete()        
        #m.go_waypoint("tote")
        #rospy.sleep(8)

       # m.go_waypoint("bin_A")
       # m.wait_till_complete()           
       # m.go_waypoint_mouth("bin_A")
       # m.wait_till_complete()    
       # m.go_relative_pose((0.12,0,0),(0,0,0,1))
       # m.wait_till_complete()  
       # rospy.sleep(2)
       # m.go_relative_pose((-0.12,0,0),(0,0,0,1))
        
        now = rospy.Time.now()
        while(not rospy.is_shutdown()):
        #    waypoint = raw_input("enter waypoint");        
        #    m.go_waypoint(waypoint)       
            #no moveit bug
            #print("------------A------------")                            
            #m.wait_till_complete("bin_A")     
            #print("------------F------------")                         
            #m.wait_till_complete("bin_F")                              
            #print("------------C------------")
            #m.wait_till_complete("bin_C")                              
            #print("------------G------------")
            #m.wait_till_complete("bin_G")
            #moveit bug
            #m.go_waypoint("bin_A")
            #m.wait_till_complete()
            #m.go_waypoint("bin_F")
            #m.wait_till_complete()
            #m.go_waypoint("bin_C")
            #m.wait_till_complete()
            #m.go_waypoint("bin_G")
            #m.wait_till_complete()    

#            m.wait_till_complete() 
            #if ((rospy.Time.now().secs - now.secs) > 20):
            #    print("B")                
            #    m.go_waypoint("bin_B")
            #    m.wait_till_complete() 
            #    now = rospy.Time.now()
            #else:
            #    print("A")   
            #    m.go_waypoint("bin_A")

        #m.wait_till_complete()           
        #m.go_waypoint_mouth("bin_A")
        #m.wait_till_complete()    
        #m.go_relative_pose((0.12,0,0),(0,0,0,1))
        #m.wait_till_complete()  
        #rospy.sleep(2)
        #m.go_relative_pose((-0.12,0,0),(0,0,0,1))


        rospy.spin()
    except rospy.ROSInterruptException:
        m.shutdown()
        pass

