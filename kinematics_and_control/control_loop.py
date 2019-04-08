#!/usr/bin/env python
#Needed functionality:
# - Publish and subscribe to the appropriate topics

# - Test/calibrate/wait to start

# - Timers
#  - Overall runtime timer
#  - Reach/travel timers

# - Request an item from the item selector
# - Signal pick failure/success to the item selector/world model

# - Send positions and poses to the motion executor, wait for response or break after limit

import rospy
import time
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import Transform,Vector3,Quaternion
from scooper_duper.msg import *
from json_handler import *
from motion_executor import motion_executor

def c_loop_vision_callback(data):
    #rospy.loginfo("Control loop recieved data from topic items_in_view")
    #rospy.loginfo(data)
    a = 1

def c_loop_gripsensor_callback(data):
    #rospy.loginfo("Control loop recieved from topic grip_sensor")
    #rospy.loginfo(data)
    global grip_state
    grip_state = data.data

#==Test routine==
print("Running connection tests...")
print("Connection tests complete.")

#==Calibration and setup==
print("Running setup and calibration...")

#Read in the pick file, set up world model.
world_model = WorldModel('/home/farscope/catkin_ws/src/scooper_duper/pick_list.json')

#Set up ros communications
rospy.init_node('control_loop', anonymous=True)
finger_pos_pub = rospy.Publisher('finger_pos', Bool, queue_size=10)
suction_state_pub = rospy.Publisher('suction_state', Bool, queue_size=10)
suction_state_pub.publish(False)
EE_pose_pub = rospy.Publisher('t_EE_pose', Transform, queue_size=10)

rospy.Subscriber("items_in_view", ItemList , c_loop_vision_callback)
rospy.Subscriber("grip_sensor", Int8 , c_loop_gripsensor_callback)
rate = rospy.Rate(0.5) # 10hz

#Setup default readings and variables
global grip_contact
grip_contact = False
mex = motion_executor()

time_limit = 600

#==Wait for operator's confirmation to start==
operator_input = raw_input("Please type 'begin' to start operations: ")
while (operator_input.lower() != "begin"):
    operator_input = raw_input("Invalid input. Please type 'begin' to start: ")

run_time = time.time() #Start time of the full operation
state = 'get_target_item'
#TODO: probably makes more sense to update all the values at the start of this loop (these are the "where am I?" variables).
#TODO: Have made assumption on grip_sensor: 0 if the gripper is open, 1 if the gripper is closed without an item, 2 if the item is closed with an item (3 if in motion?)
while ((time.time() - run_time) < time_limit) and (len(world_model.pick_list) > 0): #For 10 minutes, or until everything's picked
    if (state == 'get_target_item'):
        target_item = world_model.pick_list[0]
        target_item_bin = world_model.bins_of(target_item)[0] #Only take the first item

        op_time = time.time() #Time of start of bin move operation
        mex.go_waypoint(target_item_bin)
        state = 'move_to_bin'

    elif (state == 'move_to_bin'): #TODO: 
        bin_reached = mex.check_complete()
        #bin_reached = True
        if ((time.time() - op_time) > 15):
            world_model.pick_failure(target_item)
            state = 'get_target_item'
        elif (bin_reached == True):
            attempt_counter = 0 #Keeps track of the number of move-out-of-bin attempts (not grip attempts)
            #state = 'move_to_viewpoints' #Demo below
            suction_state_pub.publish(True)
            suck_time = time.time()
            state = 'suction_demo'
        else:
            suction_state_pub.publish(False)
            state = 'move_to_bin'

    elif (state == 'suction_demo'):
        if ((time.time() - suck_time) < 3):
            state = 'suction_demo'
        else:
            suction_state_pub.publish(False)
            world_model.pick_failure(target_item)
            state = 'get_target_item'

    elif (state == 'move_to_viewpoints'): #TODO: Handling for multiple sequential viewpoints; handling for view check and shuffle
        viewpoint_reached = True #TODO: update this with correct viewpoint reached signal (coords of bin plus offset)
        if (viewpoint_reached == True):
            time.sleep(2)         
            state = 'look_and_shuffle'
        else:
            state = 'move_to_viewpoints'

    elif (state == 'look_and_shuffle'): #TODO: This whole thing
        #Time and counter for the gripping operation
        op_time = time.time()
        op_counter = 0
        state = 'move_to_item'

    elif (state == 'move_to_item'): #TODO: Has to send actual move instruction (converted from the vision system's coordinates)
        item_reached = False #TODO: update this with correct item reached signal
        if (item_reached == True):
            state = 'attempt_grip'
        elif (time.time() - op_time < 10):
            suction_state_pub.publish(True)
            #May need a delay
            #Tell motion executor to lift
            state = 'move_to_item'        
        else:
            world_model.pick_failure(target_item)
            state = 'get_target_item'            

    elif (state == 'attempt_grip'):
        lift_reached = True
        if ((time.time() - op_time) > 100) or (op_counter >= 3):
            world_model.pick_failure(target_item)
            state = 'get_target_item'
        elif (lift_reached == False):
            state = 'attempt_grip'
        elif (grip_state == 0): #Open
            finger_pos_pub.publish(True)
            state = 'attempt_grip'
        elif (grip_state == 1): #Closed, no item
            finger_pos_pub.publish(False)
            suction_state_pub.publish(False)
            op_counter = op_counter + 1
            state = 'move_to_item'
        elif (grip_state == 2): #Closed, with item
            suction_state_pub.publish(False)
            state = 'move_to_mouth'
        #elif (grip_state == 3): #Closing
            #state = 'attempt_grip'

    elif (state == 'move_to_mouth'): #TODO: Has to send actual move instruction (mouth location, from lookup)
        mouth_reached = True #TODO: update with correct mouth reached signal        
        if (mouth_reached == False):
            state = 'move_to_mouth'
        elif (grip_contact == True):
            world_model.remove_item_from_bin(target_item, target_item_bin)
            state = 'move_to_tote'
        elif (attempt_counter < 3):
            attempt_counter = attempt_counter +1
            state = 'look_and_shuffle'
        else:
            world_model.pick_failure(target_item)
            state = 'get_target_item'

    elif (state == 'move_to_tote'): #TODO: Has to send actual move instruction (tote location, from lookup)
        tote_reached = True #TODO: update with correct tote-reached signal
        if (tote_reached == False):
            state = 'move_to_tote'
        elif (grip_contact == True):
            finger_pos_pub.pub(False)
            state = 'release_grip'
        else:
            world_model.pick_success(target_item) #Success to remove from the list
            world_model.add_item_to_bin(target_item, 'floor')
            state = 'get_target_item'

    elif (state == 'release_grip'):
        if (grip_state != 0):
            state = 'release_grip'
        else:
            world_model.pick_success(target_item)
            world_model.add_item_to_bin(target_item, 'tote')
            state = 'get_target_item'
    print(state)
    time.sleep(1)

if ((time.time() - run_time) > time_limit):
    print("Out of time; %d items remaining to pick" % len(world_model.target_items()))
else: #Not true - possibly some items were dropped
    print("All items picked successfully.")

world_model.output_to_file('output.json')
#TODO: User interaction to end operations
