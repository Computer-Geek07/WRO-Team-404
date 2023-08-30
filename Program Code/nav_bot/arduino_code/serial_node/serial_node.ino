/*
 * Rosserial serial_node to subscribe to cmd_vel and generate PWM signals for
 * the motor speed controller
 * Also publishes the PWM values to the chatter topic
 * Also turns the robot by moving the servo
 */

#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

const int max_pwm = 255;
const int motor_pwm_pin = 6;
const int motor_in1_pin = 9;
const int motor_in2_pin = 7;

Servo servo;
int servo_pin = 10;
int servo_angle_pos = 90;

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("ard_logout", &str_msg);

void cmd_messageCb(const geometry_msgs::Twist& cmd_msg) {
  // Callback to handle the cmd_vel message
  // Generate the PWM values for one motor and write them to the motor for speed
  // control Create pwm, in1, in2 variables for the motor speed controller

  // Makes the motor drive in only forward direction with a PWM value of 0-255
  int pwm = cmd_msg.linear.x * max_pwm;
  int in1 = LOW;
  int in2 = HIGH;

  // Write the values to the motor
  analogWrite(motor_pwm_pin, pwm);
  digitalWrite(motor_in1_pin, in1);
  digitalWrite(motor_in2_pin, in2);

  // Turn the robot by moving the servo
  // Servo's neutral position is 90 degrees, max left is 0 degrees, max right is
  // 180 degrees
  // Move servo to left or right according to the
  // angular velocity 0.1 rad/s = 10 degrees

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


void setup() {
  // Setup the motor pins
  pinMode(motor_pwm_pin, OUTPUT);
  pinMode(motor_in1_pin, OUTPUT);
  pinMode(motor_in2_pin, OUTPUT);
  digitalWrite(motor_in1_pin, LOW);
  digitalWrite(motor_in2_pin, LOW);
  // Setup the servo
  servo.attach(servo_pin);
  servo.write(servo_angle_pos);
  delay(100);
  // Setup the ROS node
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
