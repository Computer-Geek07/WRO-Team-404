void correctAngle(float setPoint_gyro) {
  error_gyro = currentAngle - setPoint_gyro;

  float pTerm = 0;
  float dTerm = 0;
  float iTerm = 0;

  pTerm = kp * error_gyro;
  dTerm = kd * (error_gyro - prevErrorGyro);
  totalErrorGyro += error_gyro;
  iTerm = ki * totalErrorGyro;
  correction = pTerm + iTerm + dTerm;
  if (setPoint_flag == 0) {
    if (correction > 5) {
      correction = 5;
    } else if (correction < -5) {
      correction = -5;
    }
  } else {
    if (correction > 35) {
      correction = 35;
    } else if (correction < -35) {
      correction = -35;
    }
  }

  prevErrorGyro = error_gyro;
  // if (leftTurn == 1 && rightTurn == 0) {
  //   turn(95 + correction);
  // } else if (rightTurn == 1 && leftTurn == 0) {
  //   turn(95 - correction);
  // }
  turn(91 + correction);
}

void pid_us(float setPoint_gyro, float setPointDistance, float currentDistance, bool flip) {
  float pTermGyro = 0;
  float dTermGyro = 0;
  float iTermGyro = 0;
  float pTermUltrasonic = 0.0;
  float dTermUltrasonic = 0.0;
  float iTermUltrasonic = 0.0;

  error_gyro = currentAngle - setPoint_gyro;
  totalErrorGyro += error_gyro;
  prevErrorGyro = error_gyro;

  if (!flip) {
    error_ultrasonic = distanceLeft - distanceRight;
  } else {
    error_ultrasonic = distanceRight - distanceLeft;
    // error_ultrasonic = -(currentDistance - setPointDistance);
  }
  totalErrorUltrasonic += error_ultrasonic;
  prevErrorUltrasonic = error_ultrasonic;

  pTermGyro = kp * error_gyro;
  dTermGyro = kd * (error_gyro - prevErrorGyro);
  iTermGyro = ki * totalErrorGyro;

  float correction_gyro = pTermGyro + iTermGyro + dTermGyro;

  pTermUltrasonic = kp_us * error_ultrasonic;
  dTermUltrasonic = kd_us * (error_ultrasonic - prevErrorUltrasonic);
  iTermUltrasonic = ki_us * totalErrorUltrasonic;

  float correction_ultrasonic = pTermUltrasonic + iTermUltrasonic + dTermUltrasonic;

  // correction = constrain(correction_ultrasonic+correction_gyro,56,126);
  correction = correction_ultrasonic;
  if (correction > 35) {
    correction = 35;
  } else if (correction < -35) {
    correction = -35;
  }
  turn(91 + correction);
  Serial.print("*************");
  Serial.println(correction);
}