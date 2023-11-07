// Read yaw angle from MPU
int getAngle() {
  mpu.update();
  int yawAngle = mpu.getAngleZ();  // Get the yaw angle
  return yawAngle;
}