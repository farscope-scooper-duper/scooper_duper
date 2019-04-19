#include <ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C Address 
// NOTE patrick said I2C address was Ox76, however the one I tested on was fine with the default of 0x77, see home/Arduino/libaries/Adafruit_BME280_Library/Adafruit_BME280.h
//See https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout/arduino-test for setting up of libaries 

// Vaccuum off no blockage - 1008.03//
//
///
#define P_THRESHOLD 850


ros::NodeHandle nh ;

#define VacuumPin 2 //Patricks board the arduino is connected to pin 2

bool last_msg ;

void messageCb( const std_msgs::Bool& toggle_msg){

  last_msg = toggle_msg.data ;

  if ( toggle_msg.data ) {
    digitalWrite( VacuumPin , LOW ) ;
  } else {
    digitalWrite( VacuumPin , HIGH ) ;
  }
  
}

std_msgs::Int8 grip_message;
std_msgs::Float64 sensor_message;

ros::Publisher pub_state("grip_sensor", &grip_message);
ros::Publisher pub_value("pressure_sensor", &sensor_message);

ros::Subscriber<std_msgs::Bool> sub( "suction_state" , &messageCb );

void setup() {
  
  pinMode ( VacuumPin , OUTPUT ) ;

  Serial.println(F("BME280 test"));

  bool status;
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin();  
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }

  
  pinMode ( LED_BUILTIN , OUTPUT ) ;
  digitalWrite( VacuumPin , HIGH ) ; 
  nh.initNode() ;
  nh.subscribe(sub) ;
  nh.advertise(pub_state);
  nh.advertise(pub_value);
  //Serial.begin(9600) ;

  
}


void loop() {
  // put your main code here, to run repeatedly:
  
  
  float p_reading = bme.readPressure() / 100.0f;
  sensor_message.data = p_reading;
  Serial.print(millis());
  Serial.print(" ");
  Serial.print(p_reading);
  Serial.println(" hPa");
  if (p_reading > P_THRESHOLD){ //Not blocked
    grip_message.data = 1;
  }
  else
  { //flow blocked 
    grip_message.data = 2;
  }
/* From requirements (who knows if this is actuall) what we mean)
 * 0 - Finger retracted -- Not blocked
1 - Finger extended, no item -- Not blocked
2 - Finger extended, with item -- Blocked
3 - Finger in motion --Not blocked
 */ 
  pub_state.publish(&grip_message);
  pub_value.publish(&sensor_message);

  nh.spinOnce() ;
  //Serial.println( digitalRead(LED_BUILTIN)) ;
  //delay(250 ) ;
  

  
}
