float readDistanceNew(int trigechoPin){
  int temp = analogRead(trigechoPin);
  float distance = temp * (520.0/ 4095.0) ;  // Max range upon ADC (12 bit) because Nano Esp
  return distance;

}

void ultrasounds() {
    distanceRight = readDistanceNew(triggerPinRight);
    distanceLeft = readDistanceNew(triggerPinLeft);

    // The right sensor has gone out of bounds
    // The bot has to turn right
  if (distanceRight >= OOBThresh && distanceLeft < OOBThresh) {
    rightTurn = 1;
    leftTurn = 0;
    OOB = 1;
  }
  // The left sensor has gone out of bounds
  // The bot has to turn left
   else if (distanceLeft >= OOBThresh && distanceRight < OOBThresh) {
    leftTurn = 1;
    rightTurn = 0;
    OOB = 1;
  }
  // Both sensors are within bounds
  // Bot keeps moving forward
   else  if (distanceLeft < OOBThresh && distanceRight < OOBThresh){
    OOB = 0;
  }
  prevDistanceRight = distanceRight;
  prevDistanceLeft = distanceLeft;
}