float readDistanceNew(int trigechoPin){
  int temp = analogRead(trigechoPin);
  
  float distance = temp * (520.0/ 4095.0) ;
  // Serial.print(trigechoPin);
  // Serial.print("****");
  // Serial.print(temp);
  // Serial.print("****");
  // Serial.println(distance);
  
  return distance;

}

void ultrasounds() {
  // if(leftTurn == 0){
    distanceRight = readDistanceNew(triggerPinRight);
  // }
  distanceLeft = readDistanceNew(triggerPinLeft);

  if (distanceRight >= OOBThresh && distanceLeft < OOBThresh) {
    ultrasoundRightOOB = 1;
    ultrasoundLeftOOB = 0;
    rightTurn = 1;
    leftTurn = 0;
    OOB = 1;
    // digitalWrite(greenLED, HIGH);
  } else if (distanceLeft >= OOBThresh && distanceRight < OOBThresh) {
    ultrasoundLeftOOB = 1;
    ultrasoundRightOOB = 0;
    leftTurn = 1;
    rightTurn = 0;
    OOB = 1;
    // digitalWrite(redLED, HIGH);
  } else  if (distanceLeft < OOBThresh && distanceRight < OOBThresh){
    ultrasoundRightOOB = 0;
    ultrasoundLeftOOB = 0;
    OOB = 0;
    // digitalWrite(greenLED, LOW);
  }

  // if (abs(distanceRight - prevDistanceRight) > 100) {
  //   ultrasoundRightOOB = 1;
  //   ultrasoundLeftOOB = 0;
  // } else if (abs(distanceLeft - prevDistanceLeft) > 100) {
  //   ultrasoundLeftOOB = 1;
  //   ultrasoundRightOOB = 0;
  // } else {
  //   ultrasoundRightOOB = 0;
  //   ultrasoundLeftOOB = 0;
  // }

  prevDistanceRight = distanceRight;
  prevDistanceLeft = distanceLeft;
  printDistance();
}