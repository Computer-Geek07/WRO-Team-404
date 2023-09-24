float getAngle() {
  mpu.update();
  float yawAngle = mpu.getAngleZ();  // Get the yaw angle
  return yawAngle;
}