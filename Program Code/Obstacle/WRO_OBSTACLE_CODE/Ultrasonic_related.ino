float readDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(5);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(7);
  digitalWrite(triggerPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0344827586) / 2;

  return distance;
}

void ultrasounds() {
  distanceRight = readDistance(triggerPinRight, echoPinRight);
  distanceLeft = readDistance(triggerPinLeft, echoPinLeft);

  if (distanceRight < 2) {
    digitalWrite(yellowLED, HIGH);
    digitalWrite(redLED, LOW);
  } else if (distanceLeft < 2) {
    digitalWrite(redLED, HIGH);
    digitalWrite(yellowLED, LOW);
  } else {
    digitalWrite(yellowLED, LOW);
    digitalWrite(redLED, LOW);
  }

  if (distanceRight > 200) {
    ultrasoundRightOOB = 1;
    ultrasoundLeftOOB = 0;
    digitalWrite(greenLED, HIGH);
  } else if (distanceLeft > 200) {
    ultrasoundLeftOOB = 1;
    ultrasoundRightOOB = 0;
    digitalWrite(greenLED, HIGH);
  } else {
    ultrasoundRightOOB = 0;
    ultrasoundLeftOOB = 0;
    digitalWrite(greenLED, LOW);
  }
}