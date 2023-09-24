#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>

//Gyro-related variables
MPU6050 mpu(Wire);
float globalAngle;

//Ultrasonic pin declarations
const int triggerPinLeft = 9;    // Analog output pin for the first ultrasonic sensor
const int echoPinLeft = 8;       // Analog input pin for the first ultrasonic sensor
const int triggerPinRight = 11;  // Analog output pn for the second ultrasonic sensor
const int echoPinRight = 10;     // Analog input pin for the second ultrasonic sensor

//Ultrasonic ops variables
float distanceRight;
float distanceLeft;
int ultrasoundRightOOB = 0;
int ultrasoundLeftOOB = 0;
int oobAfterTurn = 0;

//Servo variables
Servo servoMotor;

//Motor pin declarations
const int enaPin = 3;  // Enable pin
const int in1Pin = 2;  // IN1 pin
const int in2Pin = 4;  // IN2 pin

//Gyro PID variables
float error_gyro = 0;
float gyro_gain = 1;
float setPoint_gyro = 0;

//Ultrasonic PID variables
float error_us = 0;
float ultrasound_gain = 0;

//Effective PID variables
float kp = 2;     // Proportional gain
float ki = 0.00;  // Integral gain
float kd = 1.5;     // Derivative gain
float totalError = 0;
float prevError = 0;
float final_error = 0;
float pTerm = 0;
float iTerm = 0;
float dTerm = 0;
float controlOutput = 0;

//Turn related variables
int usHasRunRight = 0, usHasRunLeft = 0;
int lastRunTime = 0;
int leftFlag = 0;
int rightFlag = 0;
int turnCounter = 0;
unsigned long startTime;

// LED Pins
int greenLED = A0;
int yellowLED = A1;
int redLED = A2;

void setup() {

  Serial.begin(115200);
  Wire.begin();

  //motor setup
  pinMode(enaPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  //led setup
  pinMode(greenLED, OUTPUT);
  //  pinMode(redLED, OUTPUT);
  //  pinMode(yellowLED, OUTPUT);

  //ultrasonic setup
  pinMode(triggerPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(triggerPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);

  //servo setup
  servoMotor.attach(6);
  servoMotor.write(90);

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

  startTime = millis();
}

void loop() {
  globalAngle = getAngle();
  ultrasounds();
  allInOne(0, setPoint_gyro);
  updateFlags();
  if (turnCounter >= 12 && ultrasoundRightOOB == 0 && ultrasoundLeftOOB == 0) {
    turn(90);
    drive(0);
  } else {
    drive(120);
  }
  Serial.print("Distance Left, Distance Right : ");
  Serial.print(distanceLeft);
  Serial.print(", ");
  Serial.println(distanceRight);

}

/* Troubleshoot prints - 
printYaw();
printDistance();
printOOB();
*/