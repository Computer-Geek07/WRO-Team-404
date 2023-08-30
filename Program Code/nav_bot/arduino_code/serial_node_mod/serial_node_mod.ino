/*
   Rosserial serial_node to subscribe to cmd_vel and generate PWM signals for
   the motor speed controller
   Also publishes the PWM values to the chatter topic
   Also turns the robot by moving the servo
*/

// Import all the Header Files
#include <ros.h>

#include <Servo.h>
#include <geometry_msgs/Twist.h>
#include <nav_bot/Color_Params.h>
#include <nav_bot/Proximity_Dist.h>
#include <std_msgs/String.h>

// Color sensor pins
#define S0 13
#define S1 12
#define S2 11
#define S3 10
#define sensorOut 9

// Define the color thresholds here
#define O_lowerRed 27
#define O_upperRed 65
#define O_lowerGreen 65
#define O_upperGreen 105
#define O_lowerBlu 65
#define O_upperBlu 105

#define B_lowerRed 70
#define B_upperRed 165
#define B_lowerGreen 25
#define B_upperGreen 110
#define B_lowerBlu 15
#define B_upperBlu 80

// Motor Config
#define max_pwm 255
#define motor_pwm_pin 3
#define motor_in1_pin 2
#define motor_in2_pin 4

// Ultrasonic Sensor Pins
#define trigger_us_left A0
#define echo_us_left A1
#define trigger_us_right A4
#define echo_us_right A5

long timeElapsed = 0;
float distance = 0;

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

Servo servo;
#define servo_pin 5
int servo_angle_pos = 90;

int global_iterator = 0;
#define max_array_size 10

// Create 2 integer arrays to average the proximity sensor values
int prox_sensor_left[max_array_size];
int prox_sensor_right[max_array_size];

ros::NodeHandle nh;

std_msgs::String str_msg;
nav_bot::Color_Params color_params;
nav_bot::Proximity_Dist us_dist;

ros::Publisher chatter("ard_logout", &str_msg);
ros::Publisher color_pub("color_params", &color_params);
ros::Publisher us_pub("proximity_data", &us_dist);

void cmd_messageCb(const geometry_msgs::Twist& cmd_msg) {
  // Makes the motor drive in only forward direction with a PWM value of 0-255
  int pwm = cmd_msg.linear.x * max_pwm;
  int in1 = LOW;
  int in2 = HIGH;

  if (pwm < 0)
  {
    in1 = HIGH;
    in2 = LOW;
    pwm = pwm * -1;
  }

  // Write the values to the motor
  analogWrite(motor_pwm_pin, pwm);
  digitalWrite(motor_in1_pin, in1);
  digitalWrite(motor_in2_pin, in2);

  // Can refactor
  if (cmd_msg.angular.z > 0) {
    servo_angle_pos = 90 + (cmd_msg.angular.z * 10);
  } else if (cmd_msg.angular.z < 0) {
    servo_angle_pos = 90 + (cmd_msg.angular.z * 10);
  } else {
    servo_angle_pos = 90;
  }

  // Write the servo angle position
  servo.write(servo_angle_pos);

  // Publish the PWM and Servo Angle values to the chatter topic
  std_msgs::String str_msg;
  char buf[50];
  sprintf(buf, "PWM: %d, Servo Angle: %d", pwm, servo_angle_pos);
  str_msg.data = buf;
  chatter.publish(&str_msg);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_messageCb);

float get_distance_from_sensor(const int trigger_pin, const int echo_pin) {
  // Function to get the distance from the ultrasonic sensor
  // Trigger the ultrasonic sensor
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);

  // Get the time elapsed for the ultrasonic sensor to send and receive the
  // signal
  timeElapsed = pulseIn(echo_pin, HIGH);

  // Calculate the distance from the time elapsed
  distance = (timeElapsed * 0.034) / 2;

  // Distance has to be returned in integer, so ensure overflow doesnt happen
  if (distance > 127) {
    return 127;
  }
  return distance;
}

void redRead() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
}
void greenRead() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
}
void blueRead() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);
}

void colorSense() {
  redRead();
  greenRead();
  blueRead();
  // Condition to check if the color is orange
  if (O_lowerRed < redFrequency && redFrequency < O_upperRed &&
      O_lowerGreen < greenFrequency && greenFrequency < O_upperGreen &&
      O_lowerBlu < blueFrequency && blueFrequency < O_upperBlu) {
    color_params.green_color = 1;
    color_params.blue_color = 0;
    color_params.white_color = 0;
  }
  // Condition to check if the color is blue
  else if (B_lowerRed < redFrequency && redFrequency < B_upperRed &&
           B_lowerGreen < greenFrequency && greenFrequency < B_upperGreen &&
           B_lowerBlu < blueFrequency && blueFrequency < B_upperBlu) {
    color_params.blue_color = 1;
    color_params.green_color = 0;
    color_params.white_color = 0;
  } else {
    color_params.white_color = 1;
    color_params.blue_color = 0;
    color_params.green_color = 0;
  }
  // Publish the color values to the color_params topic
  color_pub.publish(&color_params);
}

void setup() {
  // Setup the color sensor pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  // Setting frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Setup the motor pins
  pinMode(motor_pwm_pin, OUTPUT);
  pinMode(motor_in1_pin, OUTPUT);
  pinMode(motor_in2_pin, OUTPUT);

  // Set the motor to stop
  digitalWrite(motor_in1_pin, LOW);
  digitalWrite(motor_in2_pin, LOW);

  // Setup the ultrasonic sensor pins
  pinMode(trigger_us_left, OUTPUT);
  pinMode(trigger_us_right, OUTPUT);
  pinMode(echo_us_left, INPUT);
  pinMode(echo_us_right, INPUT);

  // Setup the servo
  servo.attach(servo_pin);
  servo.write(servo_angle_pos);
  delay(100);

  // Setup the ROS node
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(us_pub);
  nh.advertise(color_pub);
  nh.subscribe(sub);
}

void loop() {
  // Get the color values from the color sensor and publish them
  colorSense();

  // Publish the values to the proximity_data topic
  us_dist.prox_dist_l = get_distance_from_sensor(trigger_us_left, echo_us_left);
  us_dist.prox_dist_r = get_distance_from_sensor(trigger_us_right, echo_us_right);
  us_pub.publish(&us_dist);

  nh.spinOnce();
  delay(1);
}
