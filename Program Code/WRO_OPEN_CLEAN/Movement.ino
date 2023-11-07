// Motor control
void drive(int power) {
  if (prevPower != power) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enaPin, power);  // Full speed
    prevPower = power;
  }
}

// Servo control
void turn(int angle) {
  if (prevAngle != angle) {
    servoMotor.write(angle);
    prevAngle = angle;
  }
}