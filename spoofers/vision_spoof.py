#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool
from scooper_duper.msg import ItemList, ItemMsg


def vision_c_loop_callback(data):
	rospy.loginfo("Vision recieved data from topic target_item");
	rospy.loginfo(data);

def spoofer():
    rospy.init_node('vision', anonymous=True)
    
    item_in_view_pub = rospy.Publisher('item_in_view', Bool, queue_size = 10)

    rospy.Subscriber("target_item", String , vision_c_loop_callback)
    
    rate = rospy.Rate(0.5) # 10hz
    state = False
    while not rospy.is_shutdown():
        #if (state == True):
        #    state = False
        #else:
        #    state = True
        item_in_view_pub.publish(state)
        rospy.loginfo("Vision info published");

        rate.sleep()

if __name__ == '__main__':
    try:
        spoofer()
    except rospy.ROSInterruptException:
        pass
