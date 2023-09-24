void allInOne(float setPoint_ultrasound, float setPoint_gyro) {

  error_gyro = globalAngle - setPoint_gyro;

  // gyro_gain = 1;
  gyro_gain = 0;
  if (abs(error_gyro) < 30) {
    gyro_gain = 0.3;
  }

  // ultrasound_gain = (1 / error_gyro) * 4;
  // if (abs(error_us) < 20) {
  //   ultrasound_gain = (1 / error_gyro) * 2;
  // }

ultrasound_gain = 1;

  if (ultrasoundRightOOB == 0 && ultrasoundLeftOOB == 0) {
    error_us = setPoint_ultrasound - (distanceLeft - distanceRight);
  } else {
    error_us = 0;
  }

  final_error = error_gyro * gyro_gain + ultrasound_gain * error_us;

  pTerm = kp * final_error;
  totalError += final_error;
  iTerm = ki * totalError;
  dTerm = kd * (final_error - prevError);

  prevError = final_error;
  controlOutput = pTerm + iTerm + dTerm;

  if (controlOutput > 35) {
    controlOutput = 35;
  } else if (controlOutput < -20) {
    controlOutput = -35;
  }

  // float mappedControlOutput = map(controlOutput, ((90*kp + 90*kd) +50), ((-90*kp + -90*kd) - 50), 20, -20);
  Serial.print("Control Output: ");
  Serial.println(controlOutput);
  turn(90 + controlOutput);
}

void updateFlags() {
  if (leftFlag == 0 && ultrasoundRightOOB == 1 && usHasRunRight == 0) {
    startTime = millis();
    setPoint_gyro = setPoint_gyro - 90;
    rightFlag = 1;
    usHasRunRight = 1;
    turnCounter++;
  } else if (rightFlag == 0 && ultrasoundLeftOOB == 1 && usHasRunLeft == 0) {
    startTime = millis();
    setPoint_gyro = setPoint_gyro + 90;
    leftFlag = 1;
    usHasRunLeft = 1;
    turnCounter++;
  }
  if (millis() - startTime >= 2400) {
    usHasRunRight = 0;
    usHasRunLeft = 0;
    oobAfterTurn = 1;
  }
  if (oobAfterTurn == 1) {
    drive(120);
    if (ultrasoundRightOOB == 0 && ultrasoundLeftOOB == 0) {
      oobAfterTurn = 0;
    }
  }
}