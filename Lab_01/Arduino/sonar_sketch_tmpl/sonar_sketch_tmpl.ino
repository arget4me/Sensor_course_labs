/*
* Template file for lab01, 2020
*/

// Includes for using ROS
#include <ros.h>
#include <sensor_msgs/Range.h>

// Define pins numbers
const int trigPin = 13; // TODO: Change this number to your value
const int echoPin = 12; // TODO: Change this number to your value 

// Declare and define variables
long duration;//[μs], 10^-6
long seq_number = 0;

// Create ROS interface
ros::NodeHandle nh;
sensor_msgs::Range range_msg;
ros::Publisher range_publisher("/ultrasound/range", &range_msg);

float speed_of_sound = 343.0f;//[m/s]
float microseconds_ratio = 0.000001f;//[10⁻6m]

void setup() {

// Set the pins up
pinMode(trigPin, OUTPUT); // Sets the trigPin
pinMode(echoPin, INPUT); // Sets the echoPin

// Set ROS node up
nh.getHardware()->setBaud(57600);
nh.initNode();
nh.advertise(range_publisher);

// Set the ROS message up
range_msg.header.frame_id="/ultrasound";
range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
range_msg.field_of_view = 15*3.1415/180.0;
range_msg.min_range = 0.02;
range_msg.max_range = 4.5;
}

void loop() {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);

// Do the measurement 
digitalWrite(trigPin, HIGH); // TODO start
delayMicroseconds(10); // How long?
digitalWrite(trigPin, LOW); // TODO stop

// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH); // TODO read

range_msg.header.seq = seq_number;
range_msg.header.stamp = nh.now();
range_msg.range = ((speed_of_sound * duration) * microseconds_ratio) / 2.0f; // [m/s]
    
// Prints the distance on the Serial Monitor
range_publisher.publish(&range_msg);
nh.spinOnce();

seq_number++;
}
