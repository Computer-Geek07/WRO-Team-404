void printYaw() {
  float yawAngle = getAngle();
  Serial.print("Yaw : ");
  Serial.println(yawAngle);
}

void printDistance() {
  distanceRight = readDistance(triggerPinRight, echoPinRight);
  distanceLeft = readDistance(triggerPinLeft, echoPinLeft);
  Serial.print("distanceLeft = ");
  Serial.print(distanceLeft);
  Serial.print("  , distanceRight = ");
  Serial.print(distanceRight);
  Serial.println(" ");
}

void printOOB() {
  ultrasounds();
  Serial.print("OOB_left = ");
  Serial.print(ultrasoundLeftOOB);
  Serial.print(" ,  OOB_right = ");
  Serial.print(ultrasoundRightOOB);
  Serial.println("  ");
}