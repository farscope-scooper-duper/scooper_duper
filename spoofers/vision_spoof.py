#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool
from scooper-duper.msg import ItemList, ItemMsg

item1 = ItemMsg()
item2 = ItemMsg()

item1.item = "mommys_helper_outlet_plugs"
item1.location = 0

item2.item = "stanley_66_052"
item2.location = 4

def spoofer():
    items_in_view_pub = rospy.Publisher('items_in_view', ItemList, queue_size = 10)

    rospy.init_node('vision_spoof', anonymous=True)
    
    rate = rospy.Rate(0.5) # 10hz
    
    while not rospy.is_shutdown():

        
        rospy.loginfo("Gripper and vacuum toggled");

        
        rate.sleep()

if __name__ == '__main__':
    try:
        spoofer()
    except rospy.ROSInterruptException:
        pass
