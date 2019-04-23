#!/usr/bin/env python
import rospy
from scooper_duper.msg import *
from geometry_msgs.msg import PoseStamped
import tf

#interface between the control loop and the vision system
class vision_interface():

    def __init__(self):
        #interface consists of sending vision request messages over a topic and the vision system responding with item poses over a different topic
        self.transformer = tf.TransformListener(True);#,rospy.Duration(10.0))
        self.vision_request_pub = rospy.Publisher('vision_request', VisionRequestMsg, queue_size=10)
        rospy.Subscriber("items_in_view", ItemList , self.vision_callback)
        self.target_item_pose = None
    def make_vision_request(self,target_item):
       
        #publish required info to the vision system, target item, bin items etc.
        while self.vision_request_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscriber to connect")
            rospy.sleep(1)

        self.target_item_pose = None

        print("making request")
        request_msg = VisionRequestMsg()
        request_msg.target_item.item = target_item
        self.vision_request_pub.publish(request_msg) 

    def get_item_position(self,in_frame = "/ee_link"):
        #converts last vision message into desired frame
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "camera"
        pose_stamped.pose = self.target_item_pose
        self.transformer.waitForTransform("/camera",in_frame, rospy.Time.now(),rospy.Duration(20.0))
        return self.transformer.transformPose(in_frame,pose_stamped)

    def vision_callback(self,data):
        #Call back to listen to the vision system's responses
        self.target_item_pose = data.items[0].pose
        print("vision callback recieved pose for " + data.items[0].item)
        #rospy.loginfo("Control loop recieved data from topic items_in_view")
        #rospy.loginfo(data)


if __name__ == '__main__':
    try:

        #test to see if the vision system responds to requests
        rospy.init_node('vision_interface_test', anonymous=True)
        interface = vision_interface()
        
        

        #rate = rospy.Rate(0.5) # 2hz
        target_item = "crayola_64_ct"
        while not rospy.is_shutdown():
            
            rospy.sleep(1)
            interface.make_vision_request(target_item)
            
            while interface.target_item_pose == None and not rospy.is_shutdown():
                #make_vision_request()
                print("Waiting for vision...")
                rospy.sleep(0.5)
            print("Recieved pose:")
            print(interface.target_item_pose)
            print("Pose in world frame:")
            pose_in_world = interface.get_item_position()
            print(pose_in_world)

            #rate.sleep()
    except rospy.ROSInterruptException:
        pass
