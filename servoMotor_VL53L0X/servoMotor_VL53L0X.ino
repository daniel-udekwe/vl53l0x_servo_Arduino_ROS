#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include "Adafruit_VL53L0X.h"
#include <std_msgs/Int16.h>
#include <Servo.h>


ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);
Adafruit_VL53L0X sensor = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

unsigned long range_timer;

// Define the servo pin
const int servoPin = 9;

// Create a Servo object
Servo myServo;

// Callback function for the subscriber
void servoCallback(const std_msgs::Int16& msg) {
  // Map the incoming value (0 to 180) to the servo range (0 to 180)
  int angle = map(msg.data, 0, 180, 0, 180);

  // Move the servo to the specified angle
  myServo.write(angle);
}

// ROS Subscriber
ros::Subscriber<std_msgs::Int16> servoSub("servo_topic", &servoCallback);

void setup() {

  // Attach the servo to the specified pin
  myServo.attach(servoPin);
  
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(pub_range);

  // Subscribe to the servo topic
  nh.subscribe(servoSub);
  
  // wait controller to be connected
  while (!nh.connected()){
    nh.spinOnce();
  }
  // if initialization failed - write message and freeze
  if (!sensor.begin()) {
    nh.logwarn("Failed to setup VL53L0X sensor");
    while(1);
  }
  nh.loginfo("VL53L0X API serial node started");
  // Wait for the connection to be established
  nh.loginfo("Servo control node initialized");
  
  // fill static range message fields
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  "ir_ranger";
  range_msg.field_of_view = 0.44; //25 degrees
  range_msg.min_range = 0.03;
  range_msg.max_range = 1.2;
}



void loop() {
  if ((millis()-range_timer) > 50){
    sensor.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        range_msg.range = (float)measure.RangeMilliMeter/1000.0f; // convert mm to m
        range_msg.header.stamp = nh.now();
        pub_range.publish(&range_msg);
    } else {
      nh.logwarn("Out of range"); // if out of range, don't send message
    }
    range_timer =  millis();    
  }
  nh.spinOnce();
}
