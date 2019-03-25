//publish to grip_sensor (Int8)
//subscribe to finger_pos (Bool)

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

void controlLoopCallback(const std_msgs::Bool& finger_message)
{
  //Default, do nothing
  ;
}

ros::Subscriber<std_msgs::Bool> sub("finger_pos", &controlLoopCallback);
std_msgs::Int8 grip_message;
ros::Publisher pub("grip_sensor, &grip_message);

void setup() 
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  int state = 0;
}

void loop() 
{
  grip_message.data = state;
  pub.publish(&grip_message);
  nh.spinOnce();

  state = state + 1;
  if (state >=4)
  {
    state = 0;
  }
  delay(10);
}
