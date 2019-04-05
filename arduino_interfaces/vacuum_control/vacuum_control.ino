#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh ;

#define VacuumPin 12
bool last_msg ;

void messageCb( const std_msgs::Bool& toggle_msg){

  last_msg = toggle_msg.data ;

  if ( toggle_msg.data ) {
    digitalWrite( VacuumPin , LOW ) ;
    digitalWrite( LED_BUILTIN, HIGH);
  } else {
    digitalWrite( VacuumPin , HIGH ) ;
    digitalWrite( LED_BUILTIN , LOW ) ;
}
  
}

ros::Subscriber<std_msgs::Bool> sub( "suction_state" , &messageCb );

void setup() {
  // put your setup code here, to run once:
  pinMode ( VacuumPin , OUTPUT ) ;
  pinMode ( LED_BUILTIN , OUTPUT ) ;
  digitalWrite( VacuumPin , HIGH ) ; 
  nh.initNode() ;
  nh.subscribe(sub) ;
  //Serial.begin(57600) ;

  
}


void loop() {
  // put your main code here, to run repeatedly:
  
  nh.spinOnce() ;
  //Serial.println( digitalRead(LED_BUILTIN)) ;
  //delay(250 ) ;
  

  
}
