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

import signal
import sys
import rospy
import time
from std_msgs.msg import String,Bool,Int8
from geometry_msgs.msg import Transform,Vector3,Quaternion
from scooper_duper.msg import *
from json_handler import *
from motion_executor import motion_executor

def signal_handler(sig, frame):
    mex.shutdown()
    suction_state_pub.publish(False)
    print("Exit called")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

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
#waitfortopic("grip_sensor")
#waitfortopic("vacuum sensor")
#waitfortopic("vision_system")

#==Calibration and setup==
print("Running setup and calibration...")

#Read in the pick file, set up world model.
world_model = WorldModel('/home/farscope/catkin_ws/src/scooper_duper/single_pick.json')

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
#TODO: Give initialisation values for everything so it doesn't mess up if the other nodes aren't transmitting yet
mex = motion_executor()

time_limit = 600


waiting_for_input = True
while waiting_for_input:
    operator_input = raw_input("Move to starting joint config (sim only) Y/N: ")
    if operator_input.lower() == "y":
        waiting_for_input = False
        mex.go_to_start()
    elif operator_input.lower() == "n":
        waiting_for_input = False

#==Wait for operator's confirmation to start==
operator_input = raw_input("Please type 'begin' to start operations: ")
while (operator_input.lower() != "begin"):
    operator_input = raw_input("Invalid input. Please type 'begin' to start: ")

run_time = time.time() #Start time of the full operation
state = 'get_target_item'
#TODO: probably makes more sense to update all the values at the start of this loop (these are the "where am I?" variables).
#TODO: Have made assumption on grip_sensor: 0 if the gripper is open, 1 if the gripper is closed without an item, 2 if the item is closed with an item (3 if in motion?)
#TODO: Add back world model management.
#TODO: Add back timing and attempt counters.

while ((time.time() - run_time) < time_limit) and (len(world_model.pick_list) > 0) and (not rospy.is_shutdown()): #For 10 minutes, or until everything's picked
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
            state = 'move_to_viewpoints'
            viewpoint = 0
            mex.go_vision_viewpoint(viewpoint, target_item_bin)
            #state = 'look_and_shuffle'
        else:
            state = 'move_to_bin'

    elif (state == 'move_to_viewpoints'): #TODO: Handling for view check and shuffle
        viewpoint_reached = mex.check_complete()
        if (viewpoint_reached == True):
            time.sleep(2)     
            viewpoint = viewpoint + 1
            if (viewpoint >= 4): #4 is number of viewpoints
                state = 'look_and_shuffle'
            else:
                mex.go_vision_viewpoint(viewpoint, target_item_bin)
                state = 'move_to_viewpoints'
        else:
            state = 'move_to_viewpoints'

    elif (state == 'look_and_shuffle'): #TODO: This whole thing
        #Time and counter for the gripping operation
        op_time = time.time()
        op_counter = 0
        state = 'move_above_item'
        #TODO: Move to the right place above the location indicated by the vision system.
        mex.go_relative_pose((0,0,0.2), (0,0,0,1))

    elif (state == 'move_above_item'): #TODO: Has to send actual move instruction (converted from the vision system's coordinates)
        above_item = mex.check_complete()
        if (above_item == True):
            suction_state_pub.publish(True)
            mex.go_relative_pose((0.12,0,0),(0,0,0,1)) ##TODO: WARNING - number is not correct in general
            dip_stage = 0 #0 is down, 1 is up            
            state = 'suction_dip'
        else:
            state = 'move_above_item'
          
    elif (state == 'suction_dip'):
        reached = mex.check_complete()
        if (reached == True):
            if (dip_stage == 0):
                dip_stage = 1
                mex.go_relative_pose((-0.12,0,0),(0,0,0,1)) ##TODO: WARNING - number is not correct in general
                state = 'suction_dip'
            else:
                finger_pos_pub.publish(True)
                state = 'attempt_grip'

    elif (state == 'attempt_grip'):
        #TODO: Add the timer stuff back in
        if (op_counter >= 2):
            world_model.pick_failure(target_item)
            suction_state_pub.publish(False)
            mex.go_waypoint(target_item_bin)
            state = 'move_to_mouth'
            #Move back to the bin mouth before moving to tote
        elif (grip_state == 0): #Open
            state = 'attempt_grip'
        elif (grip_state == 1): #Closed, no item
            finger_pos_pub.publish(False)
            suction_state_pub.publish(False)
            op_counter = op_counter + 1
            state = 'move_above_item'
            #TODO: Going to need a check here so it doesn't move while the gripper's closing
        elif (grip_state == 2): #Closed, with item
            suction_state_pub.publish(False)
            state = 'move_to_mouth'
            mex.go_waypoint(target_item_bin)
        #elif (grip_state == 3): #Opening/closing

    elif (state == 'move_to_mouth'):
        #TODO: The decision from here should depend on the grip state.
        mouth_reached = mex.check_complete()  
        if (mouth_reached == False):
            state = 'move_to_mouth'
        elif (grip_state == 0): #Open - means there's been a bad pick, we'll need to try again
            state = 'get_target_item'
        elif (grip_state == 1): #Closed, no item - mean's the item's been dropped
            pass
        elif (grip_state == 2): #Closed, item - all good, go to tote
            #mex.go_waypoint('tote') #Not yet - tote doesn't exist
            #state = 'move_to_tote'
            pass

    elif (state == 'move_to_tote'): #TODO: Has to send actual move instruction (tote location, from lookup)
        tote_reached = True #TODO: update with correct tote-reached signal
        #TODO: grip_state not grip_contact
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
if rospy.is_shutdown():
    print("Control loop aborted")    
if ((time.time() - run_time) > time_limit):
    print("Out of time; %d items remaining to pick" % len(world_model.target_items()))
else: #Not true - possibly some items were dropped
    print("All items picked successfully.")
mex.shutdown()
world_model.output_to_file('output.json')
#TODO: User interaction to end operations

#Move in front of bin
#Perform vision sweep
#Move in front of bin
#Go to above item
#Go down, suction
#Go up, grip close, suction off
#Move in front of bin
#Move to tote
