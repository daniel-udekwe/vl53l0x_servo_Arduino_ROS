#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

// Define the servo pin
const int servoPin = 9;

// Create a Servo object
Servo myServo;

// ROS NodeHandle
ros::NodeHandle nh;

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
  // Start the serial communication
  Serial.begin(115200);

  // Attach the servo to the specified pin
  myServo.attach(servoPin);

  // Initialize the ROS node
  nh.initNode();
  
  // Subscribe to the servo topic
  nh.subscribe(servoSub);

  // Wait for the connection to be established
  nh.loginfo("Servo control node initialized");
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();

  // Add any additional loop code here
}
