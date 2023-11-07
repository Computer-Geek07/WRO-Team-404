// #include <Servo.h>
#include <ESP32Servo.h> 
#include "Wire.h"
#include <MPU6050_light.h>
#include <NewPing.h>
#include "WiFi.h"

// LED Pins
#define greenLED 15
#define yellowLED 13
#define redLED 14
#define blueLED 16

// //Gyro-related variables
MPU6050 mpu(Wire);
int currentAngle;

//Ultrasonic pin declarations
const int triggerPinLeft = A1;   // Analog output pin for the first ultrasonic sensor
const int triggerPinRight = A2;  // Analog output pn for the second ultrasonic sensor

//Ultrasonic ops variables
float distanceRight;
float prevDistanceRight;
float distanceLeft;
float prevDistanceLeft;
int OOBThresh = 110;
int OOB = 0;

//Servo variables
#define servo_pin 5
Servo servoMotor;
int prevAngle = -1;

//Motor declarations
const int enaPin = 4;  // Enable pin
const int in1Pin = 3;  // IN1 pin
const int in2Pin = 2;  // IN2 pin
int prevPower = -1;
float motorSpeed = 100;

//Gyro PID variables
float error_gyro = 0;
int setPoint_gyro = 0;
int setPoint_flag = 0;

//Effective gyro PID variables
float kp = 6;  // Proportional gain
float kd = 0.5;  // Derivative gain
float ki = 0.0;
float prevErrorGyro = 0.0;
float correction = 0.0;
float totalErrorGyro = 0.0;

float error_ultrasonic = 0.0;
float totalErrorUltrasonic = 0.0;
float prevErrorUltrasonic = 0.0;


// //Turn related variables
int leftTurn = 0;
int rightTurn = 0;
int counter = 0;
unsigned long startTime;

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
  pinMode(triggerPinRight, INPUT);
  pinMode(triggerPinLeft, INPUT);
  prevDistanceRight = readDistanceNew(triggerPinRight);
  prevDistanceLeft = readDistanceNew(triggerPinLeft);

  //servo setup
  servoMotor.attach(servo_pin);
  servoMotor.write(91);

  //mpu setup
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");
  prevMillis = millis();

  // Wifi disabling
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // Start button
  // Readiness indicated by yellow LED (pin 13)
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  while (digitalRead(12) == HIGH) {
    Serial.println("Button not pressed");
    digitalWrite(13, HIGH);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  // Check for ultrasounds being Out of Bounds
  ultrasounds();

  // Read angle from gyro
  currentAngle = getAngle();

  // Stop after 3 rounds
  if (counter >= 12 && (OOB == 0)) {
    drive(0);
  } 
  else {
    drive(motorSpeed);
  }

  // If one of the ultrasonics has gone out of bounds
  if (OOB == 1) {
    // The bot is allowed to turn
    if (setPoint_flag == 0) {
      startTime = millis();

      // Turn in the relevant direction, left or right
      if (leftTurn == 1 && rightTurn == 0) {
        setPoint_gyro = setPoint_gyro + 90;  //change later for the relevant directions
      } else if (rightTurn == 1 && leftTurn == 0) {
        setPoint_gyro = setPoint_gyro - 90;
      }

      // increment turns count
      counter = counter + 1;

      // Stop the bot from turning again
      setPoint_flag = 1;
    }

    // Run gyro PID
    correctAngle(setPoint_gyro);
  } else {

    correctAngle(setPoint_gyro);  //gyro PID
    prevMillis = millis();
  }
  
  // Allow the bot to turn again is 1 second has elapsed since the last turn
  if ((millis() - startTime) > 1000) {
    setPoint_flag = 0;
  }
  }