// #include <Servo.h>
#include <ESP32Servo.h> 
#include "Wire.h"
#include <MPU6050_light.h>

// //Gyro-related variables
MPU6050 mpu(Wire);
int currentAngle;

//Ultrasonic pin declarations
const int triggerPinLeft = A7;   // Analog output pin for the first ultrasonic sensor
const int echoPinLeft = 8;       // Analog input pin for the first ultrasonic sensor
const int triggerPinRight = A6;  // Analog output pn for the second ultrasonic sensor
const int echoPinRight = 10;     // Analog input pin for the second ultrasonic sensor

//Ultrasonic ops variables
float distanceRight;
float prevDistanceRight;
float distanceLeft;
float prevDistanceLeft;
int ultrasoundRightOOB = 0;
int ultrasoundLeftOOB = 0;
int OOBThresh = 110;
int OOB = 0;
// int postTurnFlag = 0;

//Servo variables
Servo servoMotor;
int prevAngle = -1;

//Motor declarations
const int enaPin = 3;  // Enable pin
const int in1Pin = 2;  // IN1 pin
const int in2Pin = 4;  // IN2 pin
int prevPower = -1;
float motorSpeed = 150;

//Gyro PID variables
float error_gyro = 0;
// float gyro_gain = 1;
int setPoint_gyro = 0;
int setPoint_flag = 0;

// //Ultrasonic PID variables
// float error_us = 0;
// float ultrasound_gain = 0;

//Effective PID variables
float kp = 6;  // Proportional gain
float kd = 0;  // Derivative gain
float ki = 0.0;
float prevErrorGyro = 0.0;
float correction = 0.0;
float totalErrorGyro = 0.0;

float kp_us = 10;  // Proportional gain
float kd_us = 0;  // Derivative gain
float ki_us = 0.0;

float error_ultrasonic = 0.0;
float totalErrorUltrasonic = 0.0;
float prevErrorUltrasonic = 0.0;


// //Turn related variables
// int usHasRunRight = 0, usHasRunLeft = 0;
// int lastRunTime = 0;
int leftTurn = 0;
int rightTurn = 0;
int counter = 0;
unsigned long startTime;
// unsigned long startTimeTurn;
// unsigned long startTimeForward;
// int startTimeTurnFlag = 0;
// int startTimeForwardFlag = 0;

// LED Pins
int greenLED = A0;
int yellowLED = A1;
int redLED = A2;

unsigned long prevMillis;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  //motor setup
  pinMode(enaPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  //led setup
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);

  //ultrasonic setup
  // pinMode(triggerPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  // pinMode(triggerPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  prevDistanceRight = readDistanceNew(triggerPinRight);
  prevDistanceLeft = readDistanceNew(triggerPinLeft);

  //servo setup
  servoMotor.attach(11);
  servoMotor.write(91);

  //mpu setup
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");
  prevMillis = millis();
  // drive(motorSpeed);
}

void noloop(){
  ultrasounds();
}

void loop() {
  // put your main code here, to run repeatedly:
  //800 millis for oob1

  ultrasounds();

  // printDistance();
  currentAngle = getAngle();
  if (counter >= 12 && (OOB == 0)) {
    drive(0);
  } 
  else {
    drive(motorSpeed);
  }

  // if (ultrasoundRightOOB == 0 && ultrasoundLeftOOB == 0) {
  //   OOB = 0;
  //   digitalWrite(greenLED, LOW);
  // } else {
  //   OOB = 1;
  //   digitalWrite(greenLED, HIGH);
  // }

  if (OOB == 1) {
    if (setPoint_flag == 0) {
      startTime = millis();
      if (leftTurn == 1 && rightTurn == 0) {
        setPoint_gyro = setPoint_gyro + 90;  //change later for the relevant directions
      } else if (rightTurn == 1 && leftTurn == 0) {
        setPoint_gyro = setPoint_gyro - 90;
      }
      counter = counter + 1;
      // digitalWrite(yellowLED, LOW);
      setPoint_flag = 1;
    }
    // turn(95 - 90);
    // delay(800);
    // turn(95);
    // delay(200);
    correctAngle(setPoint_gyro);
    // pid_us(setPoint_gyro, distanceLeft,40,false);
    // Serial.println("Out of bounds");
  } else {
    // setPoint_flag = 0;
    // turn(95);
    // if(millis()-prevMillis >= 50){
    correctAngle(setPoint_gyro);  //eventually PID
    // pid_us(setPoint_gyro, distanceLeft,40,false);
    prevMillis = millis();
    // }
    // Serial.println(correction);
  }
  
  if ((millis() - startTime) > 1000) {
    // digitalWrite(yellowLED, HIGH);
    setPoint_flag = 0;
  }
  
  // Serial.println(setPoint_gyro);
}