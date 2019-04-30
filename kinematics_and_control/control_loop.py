#!/usr/bin/env python

from __future__ import print_function
import signal
import sys,os
import rospy
import time
import tf
#Add directory above to python path to locate constants.py
sys.path.append(os.path.join(os.path.dirname(sys.path[0]),''))

from std_msgs.msg import String,Bool,Int8,Float64
from geometry_msgs.msg import Transform,Vector3,Quaternion
from scooper_duper.msg import *
from json_handler import *
from motion_executor import motion_executor

from constants import *

def signal_handler(sig, frame):
    mex.shutdown()
    suction_state_pub.publish(False)
    print("Exit called")
    #world_model.output_to_file('{}.json'.format(time.time()))
    print("Output to file")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def c_loop_vision_callback(data):
    #rospy.loginfo("Control loop recieved data from topic items_in_view")
    #rospy.loginfo(data)
    global item_in_view
    item_in_view = data.data
    
def c_loop_pressure_callback(data):
    global pressure_state
    global vacuum_off
    if (data.data >= VACUUM_OFF_THRESH):
        pressure_state = False
        vacuum_off = True
    elif (data.data <= OBJECT_CONTACT_THRESH):
        pressure_state = True
        vacuum_off = False
    else:
        pressure_state = False
        vacuum_off = False

#Set up ros communications
rospy.init_node('control_loop', anonymous=True)

#==Test routine==
print("Running connection tests...")
print("Connection tests complete.")

#==Calibration and setup==
print("Running setup and calibration...")

#Read in the pick file, set up world model.
world_model = WorldModel( os.path.join(os.path.dirname(sys.path[0]),'vision_int_pick.json'))
mex = motion_executor()

#Set up publishing - send default values.
suction_state_pub = rospy.Publisher('suction_state', Bool, queue_size=10)
suction_state_pub.publish(False)
target_item_pub = rospy.Publisher('target_item', String, queue_size=10)
target_item = 'adventures_of_huckleberry_finn_book'
target_item_pub.publish(target_item)

#Set up subscriptions - and wait until we have received a value before continuing.
rospy.Subscriber("item_in_view", Bool , c_loop_vision_callback)
rospy.Subscriber("pressure_sensor", Float64, c_loop_pressure_callback)
rate = rospy.Rate(0.5) # 10hz

print("Waiting for pressure_sensor (from Pressure)...",end='')
sys.stdout.flush()
pressure_state = rospy.wait_for_message("pressure_sensor", Float64).data
print("OK")
print("Waiting for item_in_view (from Vision)...",end='')
sys.stdout.flush()
item_in_view = rospy.wait_for_message("item_in_view", Bool).data
print("OK")

#Check if we want to move out of singularily (should be simulation only!)
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

#The operations run until there is nothing else left on the pick list, or until the time limit has been reached.
while ((time.time() - run_time) < RUN_TIME_LIMIT) and (len(world_model.pick_list) > 0) and (not rospy.is_shutdown()):
    target_item_pub.publish(target_item)
    if (state == 'get_target_item'):
        #Pop top of pick queue
        target_item = world_model.pick_list[0]
        target_item_bin = world_model.bins_of(target_item)[0]

        op_time = time.time() #Time of start of bin move operation
        mex.go_waypoint(target_item_bin) #Tell motion exectutor to move to bin mouth
        state = 'move_to_bin'

    elif (state == 'move_to_bin'): 
        bin_reached = mex.check_complete()
        if ((time.time() - op_time) > OPERATION_TIME_LIMIT):
            world_model.pick_failure(target_item) #Signal failure
            state = 'get_target_item'
        elif (bin_reached == True):
            viewpoint = 0
            mex.go_vision_viewpoint(viewpoint, target_item_bin) #Tell motion executor to go to first viewpoint
            state = 'endoscope_sweep'
        else:
            state = 'move_to_bin'

    #Probably want checks for overall run time as well.
    elif (state == 'endoscope_sweep'):
        viewpoint_reached = mex.check_complete()
        if (viewpoint_reached == True):
            if (viewpoint == LAST_VIEWPOINT): #If we're at the last viewpoint (which should be the bin mouth again)
                viewpoint = 0
                world_model.pick_failure(target_item)
                suction_state_pub.publish(False)
                mex.go_waypoint(target_item_bin)
                state = 'move_to_mouth'
            else:
                viewpoint = viewpoint + 1 #Increment viewpoint counter
                stare_timer = time.time() #Start the timer
                state = 'endoscope_stare'
        else:
            state = 'endoscope_sweep'

    elif (state == 'endoscope_stare'):
        if (item_in_view):
            dip_timer = time.time()
            suction_state_pub.publish(True)
            mex.set_dip_pose()
            mex.go_relative_pose((0.12,0,0),(0,0,0,1),0.01, "/dip_start") ##TODO: WARNING - number is not correct in general
            dip_stage = 0 #0 is down, 1 is up        
            state = 'suction_dip'
        elif (time.time() - stare_timer) < STARE_TIME:
            state = 'endoscope_stare'
        else:
            print("About to go to:")
            print(viewpoint)
            mex.go_vision_viewpoint(viewpoint, target_item_bin)
            state = 'endoscope_sweep'

    elif (state == 'suction_dip'):
        reached = mex.check_complete()
        print(pressure_state)
        if dip_stage == 0:
            if ((pressure_state == True) or (reached == True)):
                dip_stage = 1
                mex.go_relative_pose((0,0,0),(0,0,0,1), SPEED_SCALE, "/dip_start")
                state = 'suction_dip'
            else:
                state = 'suction_dip'
        else:
            if (reached == True):
                if (pressure_state == True): #Still in contact with item (or with something)
                    mex.go_waypoint(target_item_bin)
                    state = 'move_to_mouth'
                else: #Not in contact with item
                    suction_state_pub.publish(False) #Might want checks at the start of appropriate states to see if the suction is on or not.
                    mex.go_vision_viewpoint(viewpoint, target_item_bin)
                    state = 'endoscope_sweep'
            else:
                state = 'suction_dip'


    elif (state == 'move_to_mouth'):
        mouth_reached = mex.check_complete()  
        if (mouth_reached == False):
            state = 'move_to_mouth'
        elif (pressure_state == True):
            mex.go_waypoint('tote')
            world_model.remove_item_from_bin(target_item, target_item_bin)
            state = 'move_to_tote'
        else:
            world_model.pick_failure(target_item)
            suction_state_pub.publish(False)
            state = 'get_target_item'


    elif (state == 'move_to_tote'): ## TODO: will get stuck in this state if movement fails
        tote_reached = mex.check_complete()
        if (tote_reached == False):
            state = 'move_to_tote'
        elif (pressure_state == False): #Unblocked - probably dropped the item
            pass
            world_model.pick_success(target_item) #Success to remove from the list
            world_model.add_item_to_bin(target_item, 'floor')
            state = 'get_target_item'
        else: #Blocked - assume still have the item 
            state = 'release_item'

    elif (state == 'release_item'):
        if (vacuum_off == True): #Vacuum off - assume item dropped
            #rospy.sleep(2)
            world_model.pick_success(target_item)
            world_model.add_item_to_bin(target_item, 'tote')
            state = 'get_target_item'
        else: #Could still potentially have the item
            suction_state_pub.publish(False)
            state = 'release_item'

    print(state)
    time.sleep(1)

if rospy.is_shutdown():
    print("Control loop aborted")    
if ((time.time() - run_time) > RUN_TIME_LIMIT):
    print("Out of time; %d items remaining to pick" % len(world_model.target_items()))
else: #Not true - possibly some items were dropped
    print("All items picked successfully.")

mex.shutdown()
suction_state_pub.publish(False)
world_model.output_to_file('output.json')

