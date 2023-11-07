// Gyro PID
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

  // If the bot is not turning
  // Limit the correction to 5 degrees
  // To prevent oscillations
  if (setPoint_flag == 0) {
    if (correction > 5) {
      correction = 5;
    } else if (correction < -5) {
      correction = -5;
    }
  } else { //  If bot is turning, increase the steering range
    if (correction > 35) {
      correction = 35;
    } else if (correction < -35) {
      correction = -35;
    }
  }

  prevErrorGyro = error_gyro;
  turn(90 + correction);
}