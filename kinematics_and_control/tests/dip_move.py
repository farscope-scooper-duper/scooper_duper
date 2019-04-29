#!/usr/bin/env python 
import sys,os
import rospy
import time
sys.path.append(os.path.join(os.path.dirname(sys.path[0]),''))
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(sys.path[0])),''))
from std_msgs.msg import String,Bool,Int8

from motion_executor import motion_executor
from constants import *

def c_loop_gripsensor_callback(data):
    #rospy.loginfo("Control loop recieved from topic grip_sensor")
    #rospy.loginfo(data)
    global grip_state
    grip_state = data.data


rospy.init_node('dip_test', anonymous=True)
rospy.Subscriber("grip_sensor", Int8 , c_loop_gripsensor_callback)
suction_state_pub = rospy.Publisher('suction_state', Bool, queue_size=10)
suction_state_pub.publish(False)
#
state = 'endoscope_stare'
item_in_view = True
grip_state = 0

mex = motion_executor()
mex.go_waypoint("bin_E")
mex.wait_till_complete()  
mex.go_relative_pose((0,0,0.25),(0,0,0,1),0.1)
mex.wait_till_complete()  
while not rospy.is_shutdown():
    if (state == 'endoscope_stare'):
        if (item_in_view):
            dip_timer = time.time()
            suction_state_pub.publish(True)
            mex.set_dip_pose()
            mex.go_relative_pose((0.12,0,0),(0,0,0,1),0.01,"/dip_start") ##TODO: WARNING - number is not correct in general
            dip_stage = 0 #0 is down, 1 is up 
            dip_counter = 0           
            state = 'suction_dip'

    elif (state == 'suction_dip'):
        if dip_stage == 0:
            reached = mex.check_complete() or (grip_state == 2)
            print(grip_state)
        else:
            reached = mex.check_complete()

        if (reached == True):
            print("reached bottom of dip")
            if (dip_stage == 0):
                dip_stage = 1
                mex.go_relative_pose((0,0,0),(0,0,0,1),0.01,"/dip_start") ##TODO: WARNING - number is not correct in general
                state = 'suction_dip'
            else:
                state = 'attempt_grip'

    elif (state == 'attempt_grip'):
        #TODO: Add the timer stuff back in
        if (dip_counter >= DIP_COUNTER_MAX):
            world_model.pick_failure(target_item)
            suction_state_pub.publish(False)
            mex.go_waypoint(target_item_bin)
            state = 'move_to_mouth'
        elif (grip_state == 2):
            state = 'dip success'
        elif (grip_state == 0) or (time.time() - dip_timer) > DIP_TIME_LIMIT: #no item; or timeout
            suction_state_pub.publish(True)

            dip_counter = dip_counter + 1
            dip_stage = 0
            mex.go_relative_pose((0.12,0,0),(0,0,0,1),0.01,"/dip_start")

            state = 'suction_dip'


    print(state)
    rospy.sleep(1)
