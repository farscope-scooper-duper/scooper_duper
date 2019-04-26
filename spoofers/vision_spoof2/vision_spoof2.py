#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Pose,Point,Quaternion
from scooper_duper.msg import *
import subprocess
import numpy as np
import tf 
import sys,os
items_in_view_pub = rospy.Publisher('items_in_view', ItemList, queue_size = 10)



def onVisionRequest(request):
    print("request recieved")
    if request.target_item.item =="ping": #used for a connection test to see if ethernet connection is successful
        print("ping test")
        item_list = ItemList()
        item_list.items = (ItemMsg(),)
        items_in_view_pub.publish(item_list)
    else:


        #run vision system executable (eg a python script)
        print("getting pose of: " + request.target_item.item)
        bashCommand = "python " + os.path.join(sys.path[0],"vision_system.py")
        #dummy delay        
        rospy.sleep(4)
        
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()


        #open output of vision system text file 
        f = open(os.path.join(sys.path[0],"pose.txt"),"r+")
        lines =f.readlines()
        row_count = 0
        col_count = 0
        # each line is an element in the matrix, they are listed by column so it goes
        #       element (row,column)
        #line1:  1, 1
        #line2:  1, 2
        #line3:  1, 3
        #line4:  1, 4
        #line5:  2, 1
        matrix = np.zeros((4,4))
        for x in lines:
            

            matrix[(row_count,col_count)] = float(x)        
            col_count+=1
            if (col_count >= 4):
                col_count = 0
                row_count +=1
        #item message contains the target item's pose 
        #has to be done as a list cos that's what MIT system was meant to return
        item1 = ItemMsg()

        item1.item = request.target_item.item#"mommys_helper_outlet_plugs"
        item1.location = 0
        trans = tf.transformations.translation_from_matrix(matrix)
        quat = tf.transformations.quaternion_from_matrix(matrix)

        item1.pose.position.x = trans[0]
        item1.pose.position.y = trans[1]
        item1.pose.position.z = trans[2]

        item1.pose.orientation.x = quat[0]
        item1.pose.orientation.y = quat[1]
        item1.pose.orientation.z = quat[2]
        item1.pose.orientation.w = quat[3]
        item_list = ItemList()
        item_list.items = (item1,)
        items_in_view_pub.publish(item_list)
        rospy.loginfo("Vision info published");    
   
                
if __name__ == '__main__':
    try:
        rospy.init_node('vision', anonymous=True)
        rospy.Subscriber("vision_request", VisionRequestMsg , onVisionRequest)
        rospy.spin()
        #rate = rospy.Rate(2) # 10hz
        #while not rospy.is_shutdown():
        #rate.sleep()
    except rospy.ROSInterruptException:
        pass
